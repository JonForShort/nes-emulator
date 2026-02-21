# AGENTS.md

## Purpose

This file defines project-specific guidance for coding agents working in this repository.

## Project Snapshot

- Project: `jones` NES emulator and tooling
- Language: C++20
- Build system: CMake + Make
- Main helper script: `dev/scripts/env.sh`
- Build/test environment: Docker Compose (`docker-compose.yml`, `cm/docker/*`)

## Repository Landmarks

- `source/jones`: Core emulator code (CPU, PPU, APU, memory, cartridge, controller, etc.)
- `source/frontends/sdl`: SDL frontend
- `source/frontends/flutter`: Flutter frontend and native integration
- `source/tools`: Developer/helper binaries
- `test`: Unit/trace tests
- `cmake`, `cm`: Build modules and Docker assets
- `external`: Submodules/third-party code (treat carefully)

## Standard Workflows

Prefer existing script entrypoints over ad hoc commands.
In this repository environment, always build/test through Docker Compose via `dev/scripts/env.sh` containerized commands; host-native toolchains are not available.

Containerized workflows (required):

- Build SDL in container: `./dev/scripts/env.sh jones_build_cm sdl`
- Build Flutter in container: `./dev/scripts/env.sh jones_build_cm flutter`
- Containerized tests (SDL): `./dev/scripts/env.sh jones_test_cm sdl`

## Expected Outputs

- Core/native artifacts: `out/default/jones/`
- Flutter artifacts: `out/flutter/`

## Agent Editing Rules

- Keep changes minimal and targeted.
- Do not rewrite unrelated files.
- Do not modify `external/` unless explicitly requested.
- Prefer updating existing scripts/configs over introducing new parallel tooling.
- Keep shell scripts portable and readable (bash-safe, quoted vars when practical).
- When changing Docker config, keep `docker compose` compatibility.

## Validation Expectations

When changes affect build/test/tooling:

1. Run the narrowest relevant checks first.
2. Then run at least one end-to-end containerized command from `dev/scripts/env.sh` (for example `jones_build_cm sdl` or `jones_test_cm sdl`).
3. Report exactly what was run and what passed/failed.

For script-only edits, at minimum run:

- `bash -n dev/scripts/env.sh` (if edited)

For compose edits, at minimum run:

- `docker compose config`

## Safety and Git Hygiene

- Never use destructive git operations unless explicitly requested.
- Do not revert user-owned unrelated changes.
- If unexpected file changes appear, stop and ask before proceeding.
