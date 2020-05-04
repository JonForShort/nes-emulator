FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt -y upgrade && \
    apt -y dist-upgrade && \
    apt install -y build-essential gcovr gcc-10 g++-10 gdb cmake clang-format emacs-nox && \
    apt install -y libasound2-dev texinfo file autoconf automake zlib1g-dev sudo && \
    apt autoclean -y && \
    apt remove -y && \
    apt autoremove -y

RUN groupadd --gid 1000 jones && \
    useradd --uid 1000 --gid jones --groups sudo --shell /bin/bash --create-home jones && \
    echo "jones:jones" | chpasswd

ENV HOME /home/jones

USER jones

WORKDIR /home/jones
