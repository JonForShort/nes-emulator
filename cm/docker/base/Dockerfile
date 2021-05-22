FROM ubuntu:21.04

# ---------------------------------------------------------------------------------------------
# Install dependency packages
# ---------------------------------------------------------------------------------------------

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt -y upgrade && \
    apt -y dist-upgrade && \
    apt install -y build-essential gcovr gcc g++ gdb clang-format cmake cmake-format emacs-nox && \
    apt install -y libasound2-dev texinfo file autoconf automake zlib1g-dev sudo && \
    apt autoclean -y && \
    apt remove -y && \
    apt autoremove -y \
    apt clean -y
