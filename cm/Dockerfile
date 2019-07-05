FROM ubuntu:19.10

RUN apt update && \
    apt -y upgrade && \
    apt -y dist-upgrade && \
    apt install -y build-essential gcc-8 g++-8 cmake libasound2-dev texinfo file autoconf automake && \
    apt autoclean -y && \
    apt remove -y && \
    apt autoremove -y
