FROM nes-emulator_jones-flutter:latest

# ---------------------------------------------------------------------------------------------
# Set up user environment
# ---------------------------------------------------------------------------------------------

RUN groupadd --gid 1000 jones && \
    useradd --uid 1000 --gid jones --groups sudo --shell /bin/bash --create-home jones && \
    echo "jones:jones" | chpasswd

RUN mkdir -p /opt && chown -R jones:jones /opt

ENV HOME /home/jones

USER jones

WORKDIR /home/jones
