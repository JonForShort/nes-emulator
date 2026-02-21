FROM nes-emulator/jones-flutter:local

# ---------------------------------------------------------------------------------------------
# Set up user environment
# ---------------------------------------------------------------------------------------------

RUN JONES_GID=1000 && \
    EXISTING_GROUP="$(getent group ${JONES_GID} | cut -d: -f1)" && \
    if [ -z "${EXISTING_GROUP}" ]; then \
        groupadd --gid ${JONES_GID} jones; \
        EXISTING_GROUP=jones; \
    fi && \
    if ! id -u jones >/dev/null 2>&1; then \
        if id -u 1000 >/dev/null 2>&1; then \
            useradd --gid "${EXISTING_GROUP}" --groups sudo --shell /bin/bash --create-home jones; \
        else \
            useradd --uid 1000 --gid "${EXISTING_GROUP}" --groups sudo --shell /bin/bash --create-home jones; \
        fi; \
    fi && \
    echo "jones ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/jones && \
    chmod 0440 /etc/sudoers.d/jones

RUN mkdir -p /opt && chown -R jones /opt

ENV HOME=/home/jones

USER jones

WORKDIR /home/jones
