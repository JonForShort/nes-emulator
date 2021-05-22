FROM jones_jones:latest

# ---------------------------------------------------------------------------------------------
# Install dependency packages
# ---------------------------------------------------------------------------------------------

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt -y upgrade && \
    apt -y dist-upgrade && \
    apt install -y wget unzip openjdk-8-jdk git curl clang libc++abi-11-dev libc++-11-dev ninja-build pkg-config liblzma-dev libgtk-3-dev && \
    apt autoclean -y && \
    apt remove -y && \
    apt autoremove -y

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
    yes | sdkmanager "platforms;android-29" "platform-tools" "build-tools;29.0.3" "ndk-bundle" --sdk_root=${ANDROID_SDK_ROOT}

# ---------------------------------------------------------------------------------------------
# Set up flutter
# ---------------------------------------------------------------------------------------------

ENV FLUTTER_SDK_ROOT /opt/flutter

RUN wget https://storage.googleapis.com/flutter_infra_release/releases/stable/linux/flutter_linux_2.2.0-stable.tar.xz -O ${FLUTTER_SDK_ROOT}.tar.xz && \
    tar -xf ${FLUTTER_SDK_ROOT}.tar.xz -C /opt && \
    rm -rf ${FLUTTER_SDK_ROOT}.tar.xz

ENV PATH ${PATH}:${FLUTTER_SDK_ROOT}/bin

RUN chown -R root:root /opt/flutter
