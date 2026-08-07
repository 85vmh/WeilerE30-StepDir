#!/bin/bash

# Path to halcompile. Override with: ./install_all.sh /path/to/halcompile.sh
HALCOMPILE="${1:-/home/cnc/linuxcnc_custom/bin/halcompile}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for comp in "$SCRIPT_DIR"/*.comp; do
    echo "Installing: $(basename "$comp")"
    sudo "$HALCOMPILE" --install "$comp"
done
