FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt -y upgrade && \
    apt -y dist-upgrade && \
    apt install -y build-essential gcovr gcc-10 g++-10 gdb cmake libasound2-dev texinfo file autoconf automake zlib1g-dev && \
    apt autoclean -y && \
    apt remove -y && \
    apt autoremove -y

RUN groupadd --gid 1000 jones && \
    useradd --uid 1000 --gid jones --shell /bin/bash --create-home jones

ENV HOME /home/jones

USER jones

WORKDIR /home/jones
