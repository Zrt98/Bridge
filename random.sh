#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]] || [[ ! "$1" =~ ^[0-9A-Fa-f]{12}$ ]]; then
    echo "Usage: $0 <REVERSED_MAC_HEX> [extra aoatransmitter options]" >&2
    echo "Example: $0 FFEEDDCCBBAA   # for target MAC AA:BB:CC:DD:EE:FF" >&2
    exit 1
fi

MAC="$1"
shift

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -x "./aoatransmitter" ]]; then
    BIN="./aoatransmitter"
elif [[ -x "${SCRIPT_DIR}/aoatransmitter" ]]; then
    BIN="${SCRIPT_DIR}/aoatransmitter"
elif [[ -x "${SCRIPT_DIR}/Bridge/build/aoatransmitter" ]]; then
    BIN="${SCRIPT_DIR}/Bridge/build/aoatransmitter"
elif [[ -x "${SCRIPT_DIR}/build/aoatransmitter" ]]; then
    BIN="${SCRIPT_DIR}/build/aoatransmitter"
else
    echo "Error: 'aoatransmitter' binary not found. Please build the project first:" >&2
    echo "  cd Bridge && mkdir -p build && cd build && cmake .. && make" >&2
    exit 1
fi

exec "$BIN" \
    --txgain 20 \
    --initaddr "$MAC" \
    --data "C5222B9CB3BA1C62${MAC}CDCFC72116D7C008C700C7000000A400100000000026000000,07070618E302844f03000000,07070618E302844a03000000,07070618E302844503000000,072d124d6f61f414430c14e3020806ffff02004e0a020106a3515506f7a03de6b6e483710294728162839165527380000000" \
    --rounds 20000 \
    "$@"
