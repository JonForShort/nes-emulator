FROM nes-emulator/jones-base:local

# ---------------------------------------------------------------------------------------------
# Install dependency packages
# ---------------------------------------------------------------------------------------------

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        clang \
        curl \
        git \
        libgtk-3-dev \
        liblzma-dev \
        libc++-dev \
        libc++abi-dev \
        ninja-build \
        openjdk-17-jdk \
        pkg-config && \
    rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------------------------
# Set up android sdk
# ---------------------------------------------------------------------------------------------

ENV ANDROID_SDK_ROOT=/opt/android-sdk
ENV ANDROID_SDK_HOME=${ANDROID_SDK_ROOT}
ENV ANDROID_HOME=${ANDROID_SDK_ROOT}
ENV ANDROID_SDK=${ANDROID_SDK_ROOT}
ENV ANDROID_NDK_HOME=${ANDROID_SDK_ROOT}/ndk
ENV ANDROID_NDK_ROOT=${ANDROID_SDK_ROOT}/ndk

ARG ANDROID_CMDLINE_TOOLS_VERSION=11076708

RUN mkdir -p ${ANDROID_SDK_ROOT}/cmdline-tools && \
    wget https://dl.google.com/android/repository/commandlinetools-linux-${ANDROID_CMDLINE_TOOLS_VERSION}_latest.zip -O /tmp/android-cmdline-tools.zip && \
    unzip -q /tmp/android-cmdline-tools.zip -d /tmp && \
    mv /tmp/cmdline-tools ${ANDROID_SDK_ROOT}/cmdline-tools/latest && \
    rm -rf /tmp/android-cmdline-tools.zip

ENV PATH=${PATH}:${ANDROID_SDK_ROOT}/cmdline-tools/latest/bin:${ANDROID_SDK_ROOT}/platform-tools

RUN yes | sdkmanager --update --sdk_root=${ANDROID_SDK_ROOT} && \
    yes | sdkmanager --sdk_root=${ANDROID_SDK_ROOT} --licenses && \
    sdkmanager --sdk_root=${ANDROID_SDK_ROOT} \
        "build-tools;34.0.0" \
        "platform-tools" \
        "platforms;android-34"

# ---------------------------------------------------------------------------------------------
# Set up flutter
# ---------------------------------------------------------------------------------------------

ENV FLUTTER_SDK_ROOT=/opt/flutter

ARG FLUTTER_CHANNEL=stable

RUN git clone --depth 1 --branch ${FLUTTER_CHANNEL} https://github.com/flutter/flutter.git ${FLUTTER_SDK_ROOT}

ENV PATH=${PATH}:${FLUTTER_SDK_ROOT}/bin

RUN flutter config --enable-linux-desktop && \
    flutter --version
