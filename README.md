# NES Emulator (Jones)

[![LICENSE](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Build Status](https://github.com/JonForShort/nes-emulator/workflows/Build%20Master/badge.svg)](https://github.com/JonForShort/nes-emulator/actions)

Jones is a Nintendo Entertainment System (NES) emulator and debugging-oriented tooling project implemented in modern C++.

## Overview

This repository contains:

- A core NES emulation library (`source/jones`)
- Frontends for different platforms (`source/frontends`)
- Supporting tools (`source/tools`)
- Automated tests (`test`)
- Docker-based reproducible build environments (`cm/docker`, `docker-compose.yml`)

## Repository Layout

- `source/jones`: Core emulator subsystems (CPU, PPU, APU, memory, cartridge, input, logging, utilities)
- `source/frontends/sdl`: SDL frontend
- `source/frontends/flutter`: Flutter frontend and native integration
- `source/tools`: Developer and data tooling
- `test`: CPU traces and other validation tests
- `dev/scripts/env.sh`: Main build/test helper entrypoint
- `cm/docker`: Base, Flutter, and dev Docker images

## Prerequisites

### Option 1 (Recommended): Docker

- Docker Engine with Compose plugin (`docker compose`)

### Option 2: Native Linux Toolchain

- CMake (3.14+)
- A C++20-capable compiler toolchain (`gcc`/`g++` or `clang`)
- Make
- SDL/Flutter-related dependencies if building frontends

The Docker workflow is recommended for consistent dependencies across machines and CI.

## Build and Test

All helper commands are exposed through `dev/scripts/env.sh`.

### Build (Native)

Build default target:

```bash
./dev/scripts/env.sh jones_build
```

Build SDL frontend:

```bash
./dev/scripts/env.sh jones_build_sdl
```

### Test (Native)

```bash
./dev/scripts/env.sh jones_test
```

Run tests with AddressSanitizer:

```bash
./dev/scripts/env.sh jones_test_asan
```

Generate code coverage report:

```bash
./dev/scripts/env.sh jones_test_code_coverage
```

### Build (Docker)

Build SDL variant in container:

```bash
./dev/scripts/env.sh jones_build_cm sdl
```

Build Flutter artifacts in container:

```bash
./dev/scripts/env.sh jones_build_cm flutter
```

Run containerized test flow (SDL):

```bash
./dev/scripts/env.sh jones_test_cm sdl
```

## Output Artifacts

- Native/core output: `out/default/jones/`
- Flutter output: `out/flutter/`

## Development Notes

- Project build system: CMake
- C++ standard: C++20
- Git submodules are used for external dependencies

Update submodules:

```bash
./dev/scripts/env.sh jones_update_submodules
```

Clean submodules:

```bash
./dev/scripts/env.sh jones_clean_submodules
```

## License

This project is released under the MIT License. See `LICENSE` for details.
