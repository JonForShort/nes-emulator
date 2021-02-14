FROM jones_jones:latest

# ---------------------------------------------------------------------------------------------
# Install dependency packages
# ---------------------------------------------------------------------------------------------

USER root

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt -y upgrade && \
    apt -y dist-upgrade && \
    apt install -y wget unzip default-jdk git curl clang ninja-build pkg-config liblzma-dev libgtk-3-dev && \
    apt autoclean -y && \
    apt remove -y && \
    apt autoremove -y

USER jones

# ---------------------------------------------------------------------------------------------
# Set up android sdk
# ---------------------------------------------------------------------------------------------

ENV ANDROID_SDK_ROOT /opt/android-sdk
ENV ANDROID_SDK_HOME ${ANDROID_SDK_ROOT}
ENV ANDROID_HOME     ${ANDROID_SDK_ROOT}
ENV ANDROID_SDK      ${ANDROID_SDK_ROOT}
ENV ANDROID_NDK_HOME ${ANDROID_SDK_ROOT}/ndk
ENV ANDROID_NDK_ROOT ${ANDROID_SDK_ROOT}/ndk

RUN wget https://dl.google.com/android/repository/commandlinetools-linux-6858069_latest.zip -O ${ANDROID_SDK_ROOT}.zip && \
    unzip ${ANDROID_SDK_ROOT}.zip -d ${ANDROID_SDK_ROOT} && \
    rm -rf ${ANDROID_SDK_ROOT}.zip

ENV PATH ${PATH}:${ANDROID_SDK_ROOT}/cmdline-tools/bin:${ANDROID_SDK_ROOT}/tools/bin

RUN yes | sdkmanager --update --sdk_root=${ANDROID_SDK_ROOT} && \
    yes | sdkmanager "platforms;android-29" "platform-tools" "build-tools;29.0.3" "ndk-bundle" "cmake;3.10.2.4988404" --sdk_root=${ANDROID_SDK_ROOT}

# ---------------------------------------------------------------------------------------------
# Set up flutter
# ---------------------------------------------------------------------------------------------

ENV FLUTTER_SDK_ROOT /opt/flutter

RUN wget https://storage.googleapis.com/flutter_infra/releases/stable/linux/flutter_linux_1.22.6-stable.tar.xz -O ${FLUTTER_SDK_ROOT}.tar.xz && \
    tar -xf ${FLUTTER_SDK_ROOT}.tar.xz -C /opt && \
    rm -rf ${FLUTTER_SDK_ROOT}.tar.xz

ENV PATH ${PATH}:${FLUTTER_SDK_ROOT}/bin

RUN flutter channel dev && \
    flutter upgrade && \
    flutter config --enable-linux-desktop && \
    flutter doctor