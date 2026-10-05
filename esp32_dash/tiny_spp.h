// TinySPP: a minimal Bluetooth Classic serial-port (SPP) CLIENT for the ESP32,
// written straight on top of the radio's HCI interface (VHCI).
//
// Why: Arduino's BluetoothSerial runs Espressif's full "Bluedroid" stack, which
// in the Arduino core is built with audio (A2DP), headsets (HFP), media control
// (AVRCP), BLE GATT and 4 connections - roughly 100 KB of RAM. The dashboard
// needs one thing: a serial link to one OBD adapter. This file does only that,
// in a few KB.
//
// What it implements (just enough of each layer):
//   HCI     reset, connect to a known address, legacy PIN pairing, encryption,
//           ACL data with the controller's buffer flow control
//   L2CAP   one channel to RFCOMM (PSM 3), configuration, echo/info replies
//   RFCOMM  multiplexer start, parameter negotiation with credit-based flow
//           control, modem status, one data channel (tries server channels 1..5)
// Not implemented (not needed for an ELM327): discovery by name, SDP, Secure
// Simple Pairing, being discoverable/connectable, more than one connection.
//
// Use it like BluetoothSerial (it is an Arduino Stream, so ELMduino works):
//   TinySPP bt;  bt.begin();  bt.setPin("1234", 4);
//   bt.connect(mac);  bt.connected();  bt.print("ATZ\r");  bt.read() ...
//
// Threading: the radio delivers packets on its own task; they are copied into
// a ring buffer, and ALL protocol work happens in process(), which runs inside
// whichever call you make (connect, available, read, write). Use the object
// from one task only.
#pragma once
#include <Arduino.h>
#include "esp_bt.h"

class TinySPP : public Stream {
public:
  bool debug = false;          // print protocol steps to Serial

  // ---- life cycle -----------------------------------------------------------------
  // Starts the radio in Classic-only mode. Returns false on failure.
  bool begin(const char* localName = "ESP32-OBD", bool /*isMaster*/ = true, bool /*disableBLE*/ = true)
  {
    if (_up) return true;
    _self = this;
    // (the BLE controller memory was already freed at boot, see btClassicInUse below)
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    cfg.mode = ESP_BT_MODE_CLASSIC_BT;
    cfg.bt_max_acl_conn = 1;
    esp_err_t e;
    if ((e = esp_bt_controller_init(&cfg)) != ESP_OK) { log("controller init error %d", e); return false; }
    if ((e = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) { log("controller enable error %d", e); return false; }
    static esp_vhci_host_callback_t cb = { onSendAvailable, onReceive };
    if ((e = esp_vhci_host_register_callback(&cb)) != ESP_OK) { log("vhci callback error %d", e); return false; }
    log("controller up, sending HCI reset");

    if (!cmd(0x0C03, nullptr, 0)) return fail("HCI reset");                 // Reset
    if (!cmd(0x1005, nullptr, 0)) return fail("read buffer size");          // Read Buffer Size
    uint8_t name[248] = {};
    strncpy((char*)name, localName, 247);
    cmd(0x0C13, name, sizeof(name));                                         // Write Local Name
    uint8_t cod[3] = { 0x00, 0x01, 0x00 };                                   // class: computer
    cmd(0x0C24, cod, 3);
    uint8_t scan = 0x00;                                                     // not discoverable/connectable
    cmd(0x0C1A, &scan, 1);
    uint8_t mask[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9F, 0xFF, 0x20 };   // default events + SSP ones
    cmd(0x0C01, mask, 8);
    _up = true;
    log("radio up, %d ACL buffers of %d bytes", _aclTotal, _aclMax);
    return true;
  }

  void setPin(const char* pin, uint8_t len)
  {
    _pinLen = len > 16 ? 16 : len;
    memcpy(_pin, pin, _pinLen);
  }

  // Connecting by name needs device discovery, which this client leaves out.
  bool connect(const char* /*name*/) { return false; }

  // Connect to the adapter at `mac` (most significant byte first, as printed).
  // Blocks until the serial channel is open or the attempt fails (~20 s max).
  bool connect(const uint8_t mac[6])
  {
    if (!_up) return false;
    disconnect();
    resetLink();
    for (int i = 0; i < 6; i++) _peer[i] = mac[5 - i];        // HCI wants it little-endian
    log("connecting to %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // 1. baseband connection
    uint8_t cc[13];
    memcpy(cc, _peer, 6);
    cc[6] = 0x18; cc[7] = 0xCC;          // packet types DM1/DH1/DM3/DH3/DM5/DH5
    cc[8] = 0x01; cc[9] = 0x00;          // page scan repetition R1, reserved
    cc[10] = 0x00; cc[11] = 0x00;        // clock offset unknown
    cc[12] = 0x01;                       // allow role switch
    if (!cmd(0x0405, cc, 13, true)) return fail("create connection");
    if (!waitFor([&] { return _aclUp || _aclFailed; }, 12000) || !_aclUp) return fail("baseband connect (page timeout?)");
    log("baseband connected, handle 0x%03X", _handle);

    // 2. pairing (legacy PIN) + encryption
    uint8_t h[3] = { (uint8_t)_handle, (uint8_t)(_handle >> 8), 0x01 };
    if (!cmd(0x0411, h, 2, true)) return fail("authentication request");
    if (!waitFor([&] { return _authDone || !_aclUp; }, 20000) || !_authOk) return fail("pairing (wrong PIN?)");
    if (cmd(0x0413, h, 3, true)) waitFor([&] { return _encDone || !_aclUp; }, 5000);   // optional
    log("paired%s", _encOn ? ", encrypted" : "");

    // 3. L2CAP channel to RFCOMM
    _l2Id++;
    uint8_t req[4] = { 0x03, 0x00, (uint8_t)LOCAL_CID, (uint8_t)(LOCAL_CID >> 8) };   // PSM 3, our CID
    sigSend(0x02, _l2Id, req, 4);
    if (!waitFor([&] { return _l2Connected || _l2Refused || !_aclUp; }, 10000) || !_l2Connected) return fail("L2CAP connect");
    _l2Id++;
    uint8_t cfgReq[4] = { (uint8_t)_remoteCid, (uint8_t)(_remoteCid >> 8), 0, 0 };   // no options: defaults
    sigSend(0x04, _l2Id, cfgReq, 4);
    if (!waitFor([&] { return (_l2OurCfg && _l2TheirCfg) || !_aclUp; }, 6000) || !_aclUp) return fail("L2CAP config");
    log("L2CAP channel open (remote CID 0x%04X)", _remoteCid);

    // 4. RFCOMM multiplexer
    rfSend(0, CTRL_SABM | PF, nullptr, 0, true);
    if (!waitFor([&] { return _muxUp || _muxFailed || !_aclUp; }, 6000) || !_muxUp) return fail("RFCOMM start");

    // 5. data channel: the ELM327 normally listens on server channel 1
    for (int ch = 1; ch <= 5 && !_dlcUp && _aclUp; ch++) {
      _dlci = ch << 1;
      _dlcFailed = false;
      _pnDone = false;
      uint8_t pn[8] = { (uint8_t)_dlci, 0xF0, 0, 0, (uint8_t)FRAME_MAX, 0, 0, (uint8_t)RX_CREDITS };   // ask for credit flow
      mcc(0x20, true, pn, 8);
      waitFor([&] { return _pnDone || !_aclUp; }, 3000);
      rfSend(_dlci, CTRL_SABM | PF, nullptr, 0, true);
      waitFor([&] { return _dlcUp || _dlcFailed || !_aclUp; }, 6000);
    }
    if (!_dlcUp) return fail("no RFCOMM serial channel answered");
    _rxGranted = RX_CREDITS;
    uint8_t msc[2] = { (uint8_t)((_dlci << 2) | 0x02 | 0x01), 0x8D };   // DTR/RTS/DV on
    mcc(0x38, true, msc, 2);
    waitFor([&] { return _mscIn || !_aclUp; }, 1000);
    log("serial channel open on server channel %d (credit flow %s)", _dlci >> 1, _creditFlow ? "on" : "off");
    return _dlcUp;
  }

  bool connected(int timeoutMs = 0)
  {
    process();
    if (_dlcUp || timeoutMs <= 0) return _dlcUp;
    waitFor([&] { return _dlcUp; }, timeoutMs);
    return _dlcUp;
  }

  bool hasClient() { return connected(); }

  void disconnect()
  {
    if (_aclUp) {
      uint8_t d[3] = { (uint8_t)_handle, (uint8_t)(_handle >> 8), 0x13 };   // remote user terminated
      cmd(0x0406, d, 3, true);
      waitFor([&] { return !_aclUp; }, 3000);
    }
    resetLink();
  }

  // ---- Stream ---------------------------------------------------------------------
  int available() override
  {
    flushTx();
    process();
    return (_rxHead - _rxTail) & (RX_SIZE - 1);
  }
  int read() override
  {
    if (!available()) return -1;
    uint8_t c = _rx[_rxTail];
    _rxTail = (_rxTail + 1) & (RX_SIZE - 1);
    return c;
  }
  int peek() override
  {
    if (!available()) return -1;
    return _rx[_rxTail];
  }
  // bytes are collected and sent as one frame at '\r' (end of an ELM327
  // command), when the buffer fills, or on flush()/available()
  size_t write(uint8_t c) override
  {
    if (!_dlcUp) return 0;
    _tx[_txLen++] = c;
    if (c == '\r' || _txLen >= sizeof(_tx)) flushTx();
    return 1;
  }
  size_t write(const uint8_t* p, size_t n) override
  {
    for (size_t i = 0; i < n; i++) if (!write(p[i])) return i;
    return n;
  }
  void flush() override { flushTx(); }

  // Run the protocol: handle everything the radio has sent us.
  void process()
  {
    // Not re-entrant: handling a packet can send one, and sending must not
    // start handling the next packet while this one is half done.
    if (_inProcess) return;
    _inProcess = true;
    uint8_t pkt[PKT_MAX];
    int n;
    while ((n = popPacket(pkt, sizeof(pkt))) > 0) {
      if (pkt[0] == 0x04) onEvent(pkt + 1, n - 1);
      else if (pkt[0] == 0x02) onAcl(pkt + 1, n - 1);
    }
    _inProcess = false;
    // give the adapter more credits before it runs out
    if (_dlcUp && _creditFlow && _rxGranted <= 2) {
      uint8_t more = RX_CREDITS - _rxGranted;
      _rxGranted += more;
      rfSend(_dlci, CTRL_UIH | PF, nullptr, 0, true, more);
    }
  }

  uint32_t lastError() const { return _err; }

private:
  // ---- sizes ----------------------------------------------------------------------------
  static constexpr int PKT_MAX = 1100;        // one HCI packet (ACL up to ~1021 + headers)
  static constexpr int QUEUE_SIZE = 4096;     // incoming packets waiting for process()
  static constexpr int RX_SIZE = 1024;        // serial bytes waiting to be read (power of 2)
  static constexpr int FRAME_MAX = 127;       // RFCOMM payload per frame
  static constexpr int RX_CREDITS = 7;        // frames the adapter may send before we top up
  static constexpr uint16_t LOCAL_CID = 0x0040;
  static constexpr uint8_t CTRL_SABM = 0x2F, CTRL_UA = 0x63, CTRL_DM = 0x0F, CTRL_DISC = 0x43, CTRL_UIH = 0xEF, PF = 0x10;

  // ---- radio -> us: packet ring buffer (filled on the radio's task) ---------------------
  static TinySPP* _self;
  uint8_t _q[QUEUE_SIZE];
  volatile uint32_t _qHead = 0, _qTail = 0;
  portMUX_TYPE _qMux = portMUX_INITIALIZER_UNLOCKED;

  static void onSendAvailable() {}
  static int onReceive(uint8_t* data, uint16_t len)
  {
    TinySPP* s = _self;
    if (!s || len == 0 || len > PKT_MAX) return 0;
    portENTER_CRITICAL(&s->_qMux);
    uint32_t used = s->_qHead - s->_qTail;
    if (QUEUE_SIZE - used >= (uint32_t)len + 2) {
      s->_q[s->_qHead++ % QUEUE_SIZE] = len & 0xFF;
      s->_q[s->_qHead++ % QUEUE_SIZE] = len >> 8;
      for (uint16_t i = 0; i < len; i++) s->_q[s->_qHead++ % QUEUE_SIZE] = data[i];
    }   // else dropped (should never happen at OBD data rates)
    portEXIT_CRITICAL(&s->_qMux);
    return 0;
  }
  int popPacket(uint8_t* out, int max)
  {
    int len = 0;
    portENTER_CRITICAL(&_qMux);
    if (_qHead != _qTail) {
      len = _q[_qTail++ % QUEUE_SIZE];
      len |= _q[_qTail++ % QUEUE_SIZE] << 8;
      for (int i = 0; i < len; i++) {
        uint8_t b = _q[_qTail++ % QUEUE_SIZE];
        if (i < max) out[i] = b;
      }
      if (len > max) len = 0;
    }
    portEXIT_CRITICAL(&_qMux);
    return len;
  }

  // ---- us -> radio ---------------------------------------------------------------------------
  void sendRaw(const uint8_t* p, uint16_t n)
  {
    uint32_t t0 = millis();
    while (!esp_vhci_host_check_send_available() && millis() - t0 < 200) vTaskDelay(1);
    esp_vhci_host_send_packet((uint8_t*)p, n);
  }

  // Send an HCI command and wait for its Command Complete (or Command Status
  // when `statusOnly`: the result comes later as a separate event).
  bool cmd(uint16_t opcode, const uint8_t* params, uint8_t n, bool statusOnly = false)
  {
    uint8_t p[4 + 255];
    p[0] = 0x01;
    p[1] = opcode & 0xFF;
    p[2] = opcode >> 8;
    p[3] = n;
    if (n) memcpy(p + 4, params, n);
    _cmdOpcode = opcode;
    _cmdDone = false;
    _cmdStatus = 0xFF;
    sendRaw(p, 4 + n);
    if (!waitFor([&] { return _cmdDone; }, 3000)) return false;
    return _cmdStatus == 0;
  }

  // L2CAP frame on the ACL link (waits for a free controller buffer)
  void l2Send(uint16_t cid, const uint8_t* data, uint16_t n)
  {
    if (!_aclUp) return;
    // Wait for a free controller buffer - but a reply sent from inside
    // process() can't wait (process() is what frees buffers); it goes anyway,
    // which is fine for the few small control packets that happen there.
    if (!_inProcess) waitFor([&] { return _aclFree > 0 || !_aclUp; }, 2000);
    uint8_t p[1 + 4 + 4 + 300];
    if (n > 300) return;
    uint16_t hdl = _handle | (0x2 << 12);     // first packet of an L2CAP frame, flushable
    p[0] = 0x02;
    p[1] = hdl & 0xFF; p[2] = hdl >> 8;
    p[3] = (n + 4) & 0xFF; p[4] = (n + 4) >> 8;
    p[5] = n & 0xFF; p[6] = n >> 8;
    p[7] = cid & 0xFF; p[8] = cid >> 8;
    memcpy(p + 9, data, n);
    _aclFree--;
    sendRaw(p, 9 + n);
  }

  // L2CAP signaling command (CID 1)
  void sigSend(uint8_t code, uint8_t id, const uint8_t* data, uint16_t n)
  {
    uint8_t p[4 + 64];
    p[0] = code; p[1] = id; p[2] = n & 0xFF; p[3] = n >> 8;
    memcpy(p + 4, data, n);
    l2Send(0x0001, p, 4 + n);
  }

  // RFCOMM frame. cmdFrame: a command from us (the initiator) -> C/R = 1.
  // credits >= 0: UIH with the P/F bit carries a credit byte.
  void rfSend(uint8_t dlci, uint8_t ctrl, const uint8_t* data, uint16_t n, bool cmdFrame, int credits = -1)
  {
    uint8_t f[4 + 1 + FRAME_MAX + 1];
    int i = 0;
    f[i++] = (dlci << 2) | (cmdFrame ? 0x02 : 0x00) | 0x01;
    f[i++] = ctrl;
    f[i++] = (n << 1) | 1;                    // n <= 127: one length byte
    int fcsLen = ((ctrl & ~PF) == CTRL_UIH) ? 2 : 3;
    if (credits >= 0) f[i++] = credits;
    memcpy(f + i, data, n);
    i += n;
    f[i++] = fcs(f, fcsLen);
    l2Send(_remoteCid, f, i);
  }

  // RFCOMM multiplexer control message on DLCI 0
  void mcc(uint8_t type, bool command, const uint8_t* v, uint8_t n)
  {
    uint8_t m[2 + 16];
    m[0] = (type << 2) | (command ? 0x02 : 0x00) | 0x01;
    m[1] = (n << 1) | 1;
    memcpy(m + 2, v, n);
    rfSend(0, CTRL_UIH, m, 2 + n, true);
  }

  void flushTx()
  {
    if (!_txLen || !_dlcUp) { _txLen = 0; return; }
    if (_creditFlow) waitFor([&] { return _txCredits > 0 || !_dlcUp; }, 3000);
    if (_dlcUp && (!_creditFlow || _txCredits > 0)) {
      if (_creditFlow) _txCredits--;
      rfSend(_dlci, CTRL_UIH, _tx, _txLen, true);
    }
    _txLen = 0;
  }

  // ---- incoming HCI events -----------------------------------------------------------------
  void onEvent(const uint8_t* e, int n)
  {
    if (n < 2) return;
    uint8_t code = e[0];
    const uint8_t* p = e + 2;
    switch (code) {
      case 0x0E: {                                      // Command Complete
        uint16_t op = p[1] | (p[2] << 8);
        if (op == 0x1005 && p[3] == 0) {                // Read Buffer Size
          _aclMax = p[4] | (p[5] << 8);
          _aclTotal = _aclFree = p[7] | (p[8] << 8);
        }
        if (op == _cmdOpcode) { _cmdStatus = p[3]; _cmdDone = true; }
        break;
      }
      case 0x0F: {                                      // Command Status
        uint16_t op = p[2] | (p[3] << 8);
        if (op == _cmdOpcode) { _cmdStatus = p[0]; _cmdDone = true; }
        break;
      }
      case 0x03:                                        // Connection Complete
        if (memcmp(p + 3, _peer, 6) != 0) break;
        if (p[0] == 0) { _handle = (p[1] | (p[2] << 8)) & 0x0FFF; _aclUp = true; _aclFree = _aclTotal; }
        else { _aclFailed = true; _err = p[0]; log("connection failed, HCI error 0x%02X", p[0]); }
        break;
      case 0x05:                                        // Disconnection Complete
        if (((p[1] | (p[2] << 8)) & 0x0FFF) == _handle) {
          log("disconnected (reason 0x%02X)", p[3]);
          resetLink();
        }
        break;
      case 0x16: {                                      // PIN Code Request
        uint8_t r[23] = {};
        memcpy(r, p, 6);
        r[6] = _pinLen;
        memcpy(r + 7, _pin, _pinLen);
        sendCmdNoWait(0x040D, r, 23);
        log("sent PIN");
        break;
      }
      case 0x17:                                        // Link Key Request
        if (_haveKey && memcmp(p, _keyAddr, 6) == 0) {
          uint8_t r[22];
          memcpy(r, p, 6);
          memcpy(r + 6, _key, 16);
          sendCmdNoWait(0x040B, r, 22);
        } else {
          sendCmdNoWait(0x040C, p, 6);                  // no key: pair with the PIN
        }
        break;
      case 0x18:                                        // Link Key Notification: remember it
        memcpy(_keyAddr, p, 6);
        memcpy(_key, p + 6, 16);
        _haveKey = true;
        break;
      case 0x06:                                        // Authentication Complete
        _authDone = true;
        _authOk = p[0] == 0;
        if (!_authOk) { _err = p[0]; log("pairing failed, HCI error 0x%02X", p[0]); }
        break;
      case 0x08:                                        // Encryption Change
        _encDone = true;
        _encOn = p[0] == 0 && p[3];
        break;
      case 0x13: {                                      // Number Of Completed Packets
        int handles = p[0];
        for (int i = 0; i < handles; i++) {
          uint16_t done = p[1 + handles * 2 + i * 2] | (p[2 + handles * 2 + i * 2] << 8);
          _aclFree += done;
          if (_aclFree > _aclTotal) _aclFree = _aclTotal;
        }
        break;
      }
      case 0x31: {                                      // IO Capability Request (SSP): we don't do SSP
        uint8_t r[7];
        memcpy(r, p, 6);
        r[6] = 0x18;                                    // pairing not allowed
        sendCmdNoWait(0x0434, r, 7);
        break;
      }
      default: break;                                   // role change, max slots, ... : not needed
    }
  }

  void sendCmdNoWait(uint16_t opcode, const uint8_t* params, uint8_t n)
  {
    uint8_t p[4 + 32];
    p[0] = 0x01; p[1] = opcode & 0xFF; p[2] = opcode >> 8; p[3] = n;
    memcpy(p + 4, params, n);
    sendRaw(p, 4 + n);
  }

  // ---- incoming ACL data -> L2CAP ------------------------------------------------------------
  uint8_t _l2buf[1024];
  int _l2have = 0, _l2want = 0;

  void onAcl(const uint8_t* a, int n)
  {
    if (n < 4) return;
    uint16_t hdr = a[0] | (a[1] << 8);
    if ((hdr & 0x0FFF) != _handle) return;
    uint8_t pb = (hdr >> 12) & 0x3;
    uint16_t len = a[2] | (a[3] << 8);
    const uint8_t* d = a + 4;
    if (len > n - 4) return;
    if (pb != 0x1) {                                    // start of an L2CAP frame
      _l2have = 0;
      if (len < 4) return;
      _l2want = (d[0] | (d[1] << 8)) + 4;
    }
    if (_l2have + len > (int)sizeof(_l2buf)) { _l2have = _l2want = 0; return; }
    memcpy(_l2buf + _l2have, d, len);
    _l2have += len;
    if (_l2want && _l2have >= _l2want) {
      uint16_t cid = _l2buf[2] | (_l2buf[3] << 8);
      if (cid == 0x0001) onSignal(_l2buf + 4, _l2want - 4);
      else if (cid == LOCAL_CID) onRfcomm(_l2buf + 4, _l2want - 4);
      _l2have = _l2want = 0;
    }
  }

  void onSignal(const uint8_t* s, int n)
  {
    while (n >= 4) {
      uint8_t code = s[0], id = s[1];
      uint16_t len = s[2] | (s[3] << 8);
      const uint8_t* d = s + 4;
      if (len + 4 > n) return;
      switch (code) {
        case 0x03: {                                    // Connection Response
          uint16_t dcid = d[0] | (d[1] << 8), scid = d[2] | (d[3] << 8), result = d[4] | (d[5] << 8);
          if (scid == LOCAL_CID) {
            if (result == 0) { _remoteCid = dcid; _l2Connected = true; }
            else if (result != 1) { _l2Refused = true; log("L2CAP refused (%u)", result); }
          }
          break;
        }
        case 0x04: {                                    // Configuration Request (from the adapter)
          uint8_t r[6] = { (uint8_t)_remoteCid, (uint8_t)(_remoteCid >> 8), 0, 0, 0, 0 };   // success, no options
          sigSend(0x05, id, r, 6);
          _l2TheirCfg = true;
          break;
        }
        case 0x05: {                                    // Configuration Response (to ours)
          uint16_t result = d[4] | (d[5] << 8);
          if (result == 0) _l2OurCfg = true;
          else log("L2CAP config rejected (%u)", result);
          break;
        }
        case 0x06: {                                    // Disconnection Request
          sigSend(0x07, id, d, 4);
          _l2Connected = false;
          _dlcUp = _muxUp = false;
          break;
        }
        case 0x08:                                      // Echo Request
          sigSend(0x09, id, d, len > 32 ? 32 : len);
          break;
        case 0x0A: {                                    // Information Request
          uint16_t type = d[0] | (d[1] << 8);
          if (type == 2) { uint8_t r[8] = { 2, 0, 0, 0, 0, 0, 0, 0 }; sigSend(0x0B, id, r, 8); }        // no extended features
          else if (type == 3) { uint8_t r[12] = { 3, 0, 0, 0, 0x02, 0, 0, 0, 0, 0, 0, 0 }; sigSend(0x0B, id, r, 12); }  // signaling only
          else { uint8_t r[4] = { d[0], d[1], 1, 0 }; sigSend(0x0B, id, r, 4); }                     // not supported
          break;
        }
        case 0x02: {                                    // Connection Request (adapter -> us): refuse
          uint8_t r[8] = { 0, 0, d[2], d[3], 2, 0, 0, 0 };   // PSM not supported
          sigSend(0x03, id, r, 8);
          break;
        }
        default: break;
      }
      s += 4 + len;
      n -= 4 + len;
    }
  }

  // ---- RFCOMM ----------------------------------------------------------------------------------
  void onRfcomm(const uint8_t* f, int n)
  {
    if (n < 4) return;
    uint8_t dlci = f[0] >> 2;
    uint8_t ctrl = f[1] & ~PF;
    bool pf = f[1] & PF;
    int i = 2;
    uint16_t len = f[i] >> 1;
    if (f[i++] & 1) {} else { len |= f[i++] << 7; }
    const uint8_t* d = f + i;
    int avail = n - i - 1;                              // minus FCS

    if (dlci == 0) {
      if (ctrl == CTRL_UA) _muxUp = true;
      else if (ctrl == CTRL_DM) _muxFailed = true;
      else if (ctrl == CTRL_UIH && len >= 2 && avail >= len) onMcc(d, len);
      else if (ctrl == CTRL_DISC) { rfSend(0, CTRL_UA | PF, nullptr, 0, false); _muxUp = _dlcUp = false; }
      return;
    }
    if (dlci != _dlci) {                                // a channel we didn't ask for
      if (ctrl == CTRL_SABM) rfSend(dlci, CTRL_DM | PF, nullptr, 0, false);
      return;
    }
    switch (ctrl) {
      case CTRL_UA: _dlcUp = true; break;
      case CTRL_DM: _dlcFailed = true; _dlcUp = false; break;
      case CTRL_DISC: rfSend(dlci, CTRL_UA | PF, nullptr, 0, false); _dlcUp = false; break;
      case CTRL_UIH: {
        if (pf && _creditFlow && avail >= 1) {          // first byte = credits for us
          _txCredits += d[0];                           // (the length field doesn't count this byte)
          d++; avail--;
        }
        if (len) {
          if (_creditFlow && _rxGranted > 0) _rxGranted--;
          for (int k = 0; k < len && k < avail; k++) {
            uint32_t next = (_rxHead + 1) & (RX_SIZE - 1);
            if (next == _rxTail) break;                 // full: drop (reader too slow)
            _rx[_rxHead] = d[k];
            _rxHead = next;
          }
        }
        break;
      }
      default: break;
    }
  }

  void onMcc(const uint8_t* m, int n)
  {
    uint8_t type = m[0] >> 2;
    bool command = m[0] & 0x02;
    uint8_t vlen = m[1] >> 1;
    const uint8_t* v = m + 2;
    if (vlen + 2 > n) return;
    switch (type) {
      case 0x20:                                        // PN
        if (!command) {
          _creditFlow = (v[1] & 0xF0) == 0xE0;
          _txCredits = _creditFlow ? v[7] : 0;
          _pnDone = true;
        } else {                                        // adapter proposes: accept credit flow
          uint8_t r[8];
          memcpy(r, v, 8);
          r[1] = (v[1] & 0xF0) == 0xF0 ? 0xE0 : 0x00;
          r[7] = RX_CREDITS;
          mcc(0x20, false, r, 8);
        }
        break;
      case 0x38:                                        // MSC
        if (command) { mcc(0x38, false, v, vlen); _mscIn = true; }
        break;
      case 0x24: case 0x14: case 0x08: case 0x28: case 0x18:   // RPN, RLS, test, flow on/off: echo back
        if (command) mcc(type, false, v, vlen);
        break;
      default: break;
    }
  }

  // GSM 07.10 frame check sequence (CRC-8, reversed polynomial 0xE0)
  static uint8_t fcs(const uint8_t* p, int n)
  {
    uint8_t crc = 0xFF;
    while (n--) {
      crc ^= *p++;
      for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xE0 : crc >> 1;
    }
    return 0xFF - crc;
  }

  // ---- helpers -----------------------------------------------------------------------------------
  template <typename Cond>
  bool waitFor(Cond done, uint32_t ms)
  {
    uint32_t t0 = millis();
    for (;;) {
      process();
      if (done()) return true;
      if (millis() - t0 > ms) return false;
      vTaskDelay(1);
    }
  }

  void resetLink()
  {
    _aclUp = _aclFailed = _authDone = _authOk = _encDone = _encOn = false;
    _l2Connected = _l2Refused = _l2OurCfg = _l2TheirCfg = false;
    _muxUp = _muxFailed = _dlcUp = _dlcFailed = _pnDone = _mscIn = _creditFlow = false;
    _txCredits = 0;
    _rxGranted = 0;
    _txLen = 0;
    _l2have = _l2want = 0;
  }

  bool fail(const char* what)
  {
    log("FAILED: %s", what);
    return false;
  }

  void log(const char* fmt, ...)
  {
    if (!debug) return;
    char b[96];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    Serial.printf("[SPP] %s\n", b);
  }

  // ---- state -----------------------------------------------------------------------------------
  bool _up = false;
  bool _inProcess = false;
  uint8_t _pin[16] = { '1', '2', '3', '4' };
  uint8_t _pinLen = 4;
  uint8_t _peer[6] = {};
  uint16_t _handle = 0x0FFF;
  uint16_t _cmdOpcode = 0;
  volatile bool _cmdDone = false;
  uint8_t _cmdStatus = 0;
  int _aclMax = 0, _aclTotal = 1, _aclFree = 1;
  bool _aclUp = false, _aclFailed = false, _authDone = false, _authOk = false, _encDone = false, _encOn = false;
  bool _haveKey = false;
  uint8_t _keyAddr[6], _key[16];
  uint8_t _l2Id = 0;
  uint16_t _remoteCid = 0;
  bool _l2Connected = false, _l2Refused = false, _l2OurCfg = false, _l2TheirCfg = false;
  bool _muxUp = false, _muxFailed = false, _dlcUp = false, _dlcFailed = false, _pnDone = false, _mscIn = false;
  bool _creditFlow = false;
  uint8_t _dlci = 2;
  int _txCredits = 0, _rxGranted = 0;
  uint8_t _tx[FRAME_MAX];
  size_t _txLen = 0;
  uint8_t _rx[RX_SIZE];
  uint32_t _rxHead = 0, _rxTail = 0;
  uint32_t _err = 0;
};

TinySPP* TinySPP::_self = nullptr;

// The Arduino core frees all Bluetooth memory at boot unless a library says
// it's in use (BluetoothSerial normally does). Classic yes, BLE no: the core
// then keeps the Classic controller memory and frees the BLE part for us.
extern "C" bool btClassicInUse(void) { return true; }
