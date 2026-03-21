#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for comp in "$SCRIPT_DIR"/*.comp; do
    echo "Installing: $(basename "$comp")"
    sudo halcompile --install "$comp"
done
