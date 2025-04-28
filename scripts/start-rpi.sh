#!/bin/bash

# handle `## rpi-only ##` in pixi.toml, which is needed for the `rpi-lgpio` dependency
# it can only be installed and used on the Pi, because it depends on the `lgpio` library
sed -i 's/## rpi-only ## //g' pixi.toml

pixi run build-ros
pixi run launch
