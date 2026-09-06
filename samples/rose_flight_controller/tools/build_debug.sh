#!/usr/bin/env bash
# Alias for the 'viz' preset (visualize-only, motors OFF, continuous -- for tethered state_viz).
# All presets now live in tools/build.sh. The motors-on continuous bench test is now the 'feel'
# preset:  tools/build.sh feel [motor_duty].
exec "$(dirname "$0")/build.sh" viz
