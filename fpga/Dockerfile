# Quartus Prime Lite 23.1.1 (command line) for the MAX1000's MAX 10 FPGA.
# x86-64 Linux image; on Apple Silicon, OrbStack runs it through Rosetta.
#
# Needs these two files in ./installers (free Altera account to download):
#   QuartusLiteSetup-23.1std.1.993-linux.run
#   max10-23.1std.1.993.qdz
# Build: ./quartus.sh build
FROM --platform=linux/amd64 ubuntu:22.04

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ca-certificates locales make perl python3 \
      libglib2.0-0 libfontconfig1 libfreetype6 libx11-6 libxext6 libxrender1 libsm6 libice6 \
      libxft2 libxtst6 libxi6 libxau6 libxdmcp6 libncurses5 libtinfo5 libudev1 libusb-1.0-0 \
 && locale-gen en_US.UTF-8 && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# The installer picks up the .qdz device file sitting next to it.
COPY installers/ /tmp/installers/
RUN chmod +x /tmp/installers/QuartusLiteSetup-*.run \
 && /tmp/installers/QuartusLiteSetup-23.1std.1.993-linux.run \
      --mode unattended --unattendedmodeui none --accept_eula 1 \
      --installdir /opt/intelFPGA_lite/23.1std \
 && rm -rf /tmp/installers /opt/intelFPGA_lite/23.1std/uninstall

ENV PATH=/opt/intelFPGA_lite/23.1std/quartus/bin:$PATH
WORKDIR /work
CMD ["bash"]
