FROM ubuntu:20.04

RUN apt update && \
    apt -y upgrade && \
    apt -y dist-upgrade && \
    apt install -y build-essential gcovr gcc-10 g++-10 cmake libasound2-dev texinfo file autoconf automake zlib1g-dev && \
    apt autoclean -y && \
    apt remove -y && \
    apt autoremove -y
