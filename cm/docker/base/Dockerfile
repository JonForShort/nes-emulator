FROM ubuntu:24.04

# ---------------------------------------------------------------------------------------------
# Install dependency packages
# ---------------------------------------------------------------------------------------------

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        autoconf \
        automake \
        build-essential \
        ca-certificates \
        clang-format \
        cmake \
        curl \
        emacs-nox \
        file \
        g++ \
        gcc \
        gcovr \
        gdb \
        git \
        libasound2-dev \
        python3 \
        python3-pip \
        sudo \
        texinfo \
        unzip \
        wget \
        xz-utils \
        zlib1g-dev && \
    pip3 install --break-system-packages cmakelang && \
    rm -rf /var/lib/apt/lists/*
