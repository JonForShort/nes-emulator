FROM ubuntu:21.04

# ---------------------------------------------------------------------------------------------
# Install dependency packages
# ---------------------------------------------------------------------------------------------

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt -y upgrade && \
    apt -y dist-upgrade && \
    apt install -y build-essential gcovr gcc g++ gdb cmake clang-format emacs-nox && \
    apt install -y libasound2-dev texinfo file autoconf automake zlib1g-dev sudo && \
    apt autoclean -y && \
    apt remove -y && \
    apt autoremove -y

# ---------------------------------------------------------------------------------------------
# Set up user environment
# ---------------------------------------------------------------------------------------------

RUN groupadd --gid 1000 jones && \
    useradd --uid 1000 --gid jones --groups sudo --shell /bin/bash --create-home jones && \
    echo "jones:jones" | chpasswd

RUN mkdir -p /opt && chown jones:jones /opt

ENV HOME /home/jones

USER jones

WORKDIR /home/jones
