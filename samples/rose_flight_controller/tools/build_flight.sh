#!/usr/bin/env bash
# Alias for the 'flight' preset. All build presets now live in tools/build.sh (single source of
# truth). Full flight parameter rationale: docs/FLIGHT_BUILD.md.
exec "$(dirname "$0")/build.sh" flight
