
ARG VARIANT="bullseye"
FROM mcr.microsoft.com/vscode/devcontainers/rust:1-${VARIANT}

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends gdb-multiarch binutils-arm-none-eabi gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential wget openocd stlink-tools 

USER vscode
RUN     rustup target add thumbv7em-none-eabihf \
    && rustup component add llvm-tools-preview \
    &&  cargo install cargo-binutils
