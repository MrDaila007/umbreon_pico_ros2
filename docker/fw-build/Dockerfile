FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates gcc-arm-none-eabi libnewlib-arm-none-eabi \
      cmake ninja-build gcc g++ make python3 python3-pip git gcovr \
      pkg-config libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch 2.2.0 --recurse-submodules --shallow-submodules \
      https://github.com/raspberrypi/pico-sdk /opt/pico-sdk

# The RP2350_ARM_NTZ port lives in the Community-Supported-Ports submodule of this tag.
RUN git clone --depth 1 --branch V11.2.0 --recurse-submodules --shallow-submodules \
      https://github.com/FreeRTOS/FreeRTOS-Kernel /opt/FreeRTOS-Kernel \
    && test -d /opt/FreeRTOS-Kernel/portable/ThirdParty/Community-Supported-Ports/GCC/RP2350_ARM_NTZ

ENV PICO_SDK_PATH=/opt/pico-sdk FREERTOS_KERNEL_PATH=/opt/FreeRTOS-Kernel

RUN git clone --depth 1 --branch 2.2.0 https://github.com/raspberrypi/picotool /tmp/picotool \
    && cmake -S /tmp/picotool -B /tmp/picotool/build -G Ninja -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/picotool/build \
    && cmake --install /tmp/picotool/build \
    && rm -rf /tmp/picotool

WORKDIR /work
