#!/usr/bin/env bash
set -euo pipefail

RULE_FILE=/etc/udev/rules.d/70-ros2bot-master.rules
RULE='SUBSYSTEM=="tty", KERNEL=="ttyUSB[0-9]*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{bcdDevice}=="8134", SYMLINK+="r2bserial", GROUP="dialout", MODE="0660"'
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
CHECK_ONLY=false

usage() {
    printf 'Usage: %s [--check]\n' "$0"
    printf '  --check  Diagnose without changing udev rules or opening the board.\n'
}

fail() {
    printf 'ERROR: %s\n' "$*" >&2
    exit 1
}

if [[ $# -gt 1 ]]; then
    usage >&2
    exit 2
fi
case "${1:-}" in
    --check) CHECK_ONLY=true ;;
    --help|-h) usage; exit 0 ;;
    '') ;;
    *) usage >&2; exit 2 ;;
esac

[[ $(uname -s) == Linux ]] || fail 'This setup requires Linux and udev.'
command -v udevadm >/dev/null || fail 'udevadm is missing; install systemd-udev.'
shopt -s nullglob
ports=(/dev/ttyUSB*)
matches=()
for port in "${ports[@]}"; do
    properties=$(udevadm info --query=property --name="$port") || continue
    if grep -qx 'ID_VENDOR_ID=1a86' <<< "$properties" &&
       grep -qx 'ID_MODEL_ID=7523' <<< "$properties" &&
       grep -qx 'ID_REVISION=8134' <<< "$properties"; then
        matches+=("$port")
        printf 'Robot adapter candidate: %s\n' "$port"
    fi
done

if (( ${#matches[@]} == 0 )); then
    if command -v lsusb >/dev/null && lsusb -d 1a86:7523 | grep -q .; then
        fail 'CH340 USB device detected but no tty with revision 8134. Check lsusb -t; if Driver=[none], install a ch341 driver built for the running kernel, then retry.'
    fi
    fail 'No CH340 adapter with revision 8134 found. Connect and power the robot board, then check lsusb -t and udevadm info -a -n /dev/ttyUSBn.'
fi
(( ${#matches[@]} == 1 )) || fail 'Multiple adapters match 1a86:7523 revision 8134; an automatic port-independent alias would be ambiguous.'
port=${matches[0]}
properties=$(udevadm info --query=property --name="$port")
grep -qx 'ID_USB_DRIVER=ch341' <<< "$properties" || fail "$port is not bound to ch341. Check lsusb -t and the kernel module."

for rule in /etc/udev/rules.d/*.rules /run/udev/rules.d/*.rules /usr/lib/udev/rules.d/*.rules /lib/udev/rules.d/*.rules; do
    [[ $rule == "$RULE_FILE" ]] && continue
    if [[ -r $rule ]]; then
        rule_text=$(<"$rule")
    else
        $CHECK_ONLY && fail "Cannot inspect $rule; run 'sudo bash $0 --check' to complete the read-only check."
        command -v sudo >/dev/null || fail "Cannot inspect $rule without sudo."
        rule_text=$(sudo cat -- "$rule") || fail "Cannot inspect $rule even with sudo."
    fi
    if grep -Eq 'SYMLINK\+?="?r2bserial"?' <<< "$rule_text"; then
        fail "Conflicting alias in $rule. Back up that file and remove only its r2bserial assignment (the old CP210x rule targeted the lidar), then rerun."
    fi
done

if $CHECK_ONLY; then
    printf 'Ready: %s is bound to ch341; no conflicting r2bserial rule found.\n' "$port"
    if [[ -e /dev/r2bserial ]]; then
        printf 'Current alias: %s\n' "$(readlink -f /dev/r2bserial)"
        [[ $(readlink -f /dev/r2bserial) == "$port" ]] || fail 'Existing alias points to another device. Reconnect the robot USB cable after updating the rules.'
    fi
    exit 0
fi

command -v sudo >/dev/null || fail 'sudo is required to install the udev rule.'
if [[ ! -f $RULE_FILE ]] || [[ $(<"$RULE_FILE") != "$RULE" ]]; then
    if [[ -e $RULE_FILE ]]; then
        fail "$RULE_FILE differs from the expected rule; review and back it up before replacing it."
    fi
    printf '%s\n' "$RULE" | sudo install -m 0644 /dev/stdin "$RULE_FILE" || fail 'Could not install the udev rule.'
fi
sudo udevadm control --reload-rules || fail 'Could not reload udev rules.'
sudo udevadm trigger --action=add --subsystem-match=tty --sysname-match="${port##*/}" || fail 'Could not reapply the udev rule; reconnect the robot USB cable.'
sudo udevadm settle || fail 'udev did not settle; reconnect the robot USB cable.'

[[ -e /dev/r2bserial ]] || fail 'The alias was not created. Reconnect the robot USB cable and inspect udevadm info -a -n the current tty.'
[[ $(readlink -f /dev/r2bserial) == "$port" ]] || fail "The alias points to $(readlink -f /dev/r2bserial), not $port. Inspect other udev rules."
printf 'Alias ready: /dev/r2bserial -> %s\n' "$port"

if [[ -x $SCRIPT_DIR/.venv/bin/python ]]; then
    python_cmd=$SCRIPT_DIR/.venv/bin/python
elif command -v python3 >/dev/null; then
    python_cmd=$(command -v python3)
else
    fail 'No Python interpreter found. Install Python, pyserial and ros2bot_master_lib, then run test_master_lib.py get_version.'
fi
if ! output=$("$python_cmd" "$SCRIPT_DIR/test_master_lib.py" get_version --port /dev/r2bserial --debug 2>&1); then
    printf '%s\n' "$output"
    fail 'Alias is correct but the version test failed. Install the library and pyserial in the selected Python environment, check dialout access, board power, and the USB cable.'
fi
printf '%s\n' "$output"
version=$(awk '/^get_version: / {print $2}' <<< "$output")
if [[ ! $version =~ ^[0-9]+([.][0-9]+)?$ ]] || [[ $version == 0 || $version == 0.0 ]]; then
    fail 'No valid board version reply. Check board power, USB cable, and that the CH340 is connected to the robot board.'
fi
printf 'Setup complete: board firmware %s.\n' "$version"