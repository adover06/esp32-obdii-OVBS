// Command parser: SPI bytes -> pixel writes into the back buffer, and swaps.
//
// Every transaction (CS low ... CS high) starts with a 4-byte command header
// [cmd, 0, 0, 0]; the padding keeps the pixel data 4-byte aligned for the
// ESP32's DMA.
//   'W' (0x57)  + x, y, w, h (16-bit little endian each) + w*h pixel bytes,
//               row by row. Writes the rectangle into the back buffer;
//               anything outside 360x240 is dropped. Extra bytes are ignored.
//   'S' (0x53)  show the back buffer at the next vertical blank. Poll the
//               READY pin / status flag before drawing the next frame.
//   anything else (e.g. 'Q' status read, 'X' link test) is ignored, but its
//   bytes still count toward the CRC returned in the status word.
//
// Python sketch of what this does with each incoming byte:
//     if first_byte_of_transaction:
//         state = "HDR" if byte == ord('W') else "IDLE"
//         if byte == ord('S'): request_swap()
//     elif state == "HDR":            # 3 padding bytes, then x, y, w, h
//         collect header; when done: cx, cy = x, y; state = "PIX"
//     elif state == "PIX" and cy < y + h:
//         if cx < 360 and cy < 240: fifo.push((back_buf, cy, cx, byte))
//         cx += 1
//         if cx == x + w: cx = x; cy += 1    # next row of the rectangle
module cmd_parser (
  input  wire        clk,
  input  wire        rst,
  input  wire        enable,       // accept commands (frame buffers ready)
  input  wire        back,         // back buffer index
  input  wire        byte_valid,
  input  wire [7:0]  byte_data,
  input  wire        byte_first,
  output reg         push,
  output reg  [25:0] push_data,    // {buf, line, col[7:0], hi, byte}
  output reg         swap_cmd
);
  localparam ST_IDLE = 2'd0, ST_HDR = 2'd1, ST_PIX = 2'd2;

  reg [1:0]  st;
  reg [3:0]  idx;                  // header byte index (3 pad + 8 params)
  reg [15:0] x0, y0, w, hgt;
  reg [16:0] cx, cy, x_end, y_end;
  reg        buf_sel;

  wire in_rect = (cy < y_end);
  wire on_pic  = (cx < 17'd360) && (cy < 17'd240);

  always @(posedge clk) begin
    push     <= 1'b0;
    swap_cmd <= 1'b0;
    if (rst) begin
      st <= ST_IDLE;
    end else if (byte_valid) begin
      if (byte_first) begin
        idx <= 4'd0;
        st  <= ST_IDLE;
        if (byte_data == 8'h57) begin
          st      <= ST_HDR;
          buf_sel <= back;
        end else if (byte_data == 8'h53 && enable) begin
          swap_cmd <= 1'b1;
        end
      end else begin
        case (st)
          ST_HDR: begin
            idx <= idx + 4'd1;
            case (idx)
              4'd3:  x0[7:0]   <= byte_data;
              4'd4:  x0[15:8]  <= byte_data;
              4'd5:  y0[7:0]   <= byte_data;
              4'd6:  y0[15:8]  <= byte_data;
              4'd7:  w[7:0]    <= byte_data;
              4'd8:  w[15:8]   <= byte_data;
              4'd9:  hgt[7:0]  <= byte_data;
              4'd10: begin
                hgt[15:8] <= byte_data;
                cx    <= {1'b0, x0};
                cy    <= {1'b0, y0};
                x_end <= {1'b0, x0} + {1'b0, w};
                y_end <= {1'b0, y0} + {1'b0, byte_data, hgt[7:0]};
                st    <= (w == 16'd0 || {byte_data, hgt[7:0]} == 16'd0) ? ST_IDLE : ST_PIX;
              end
              default: ;   // padding bytes 0..2
            endcase
          end
          ST_PIX: begin
            if (in_rect) begin
              if (on_pic && enable) begin
                push      <= 1'b1;
                push_data <= {buf_sel, cy[7:0], cx[8:1], cx[0], byte_data};
              end
              if (cx + 17'd1 == x_end) begin
                cx <= {1'b0, x0};
                cy <= cy + 17'd1;
              end else begin
                cx <= cx + 17'd1;
              end
            end
          end
          default: ;
        endcase
      end
    end
  end
endmodule
