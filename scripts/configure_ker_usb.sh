#!/usr/bin/env bash
# Grant the selected local user access to the KER USB device, not all USB devices.
set -euo pipefail

usage() {
  echo "usage: sudo bash $0 [USER]" >&2
  echo "       bash $0 --print-rule USER" >&2
}

PRINT_ONLY=0
if [[ "${1:-}" == "--print-rule" ]]; then
  PRINT_ONLY=1
  shift
fi
if [[ "$#" -gt 1 ]]; then usage; exit 2; fi
KER_USER="${1:-${SUDO_USER:-}}"
if [[ ! "${KER_USER}" =~ ^[a-zA-Z_][a-zA-Z0-9_.-]*\$?$ ]] || [[ "${KER_USER}" == "root" ]]; then
  echo "[ker-usb][ERROR] Specify the regular user who runs ./app.sh." >&2
  usage
  exit 2
fi
id "${KER_USER}" >/dev/null

print_rule() {
  printf '%s\n' '# OpenArm KER (M5Stack CoreS3, normal USB vendor mode).'
  printf 'SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="303a", ATTR{idProduct}=="4002", OWNER="%s", MODE="0660", TAG+="uaccess"\n' "${KER_USER}"
}
if [[ "${PRINT_ONLY}" == "1" ]]; then
  print_rule
  exit 0
fi
if [[ "${EUID}" -ne 0 ]]; then
  echo "[ker-usb][ERROR] Run: sudo bash $0 ${KER_USER}" >&2
  exit 1
fi
command -v udevadm >/dev/null
KER_RULE="/etc/udev/rules.d/70-openarm-ker.rules"
KER_RULE_TMP="$(mktemp)"
trap 'rm -f "${KER_RULE_TMP}"' EXIT
print_rule > "${KER_RULE_TMP}"
install -o root -g root -m 0644 "${KER_RULE_TMP}" "${KER_RULE}"
udevadm control --reload-rules
# Reapply only to KER devices already attached. Do not touch CAN, cameras, or firmware.
udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=303a --attr-match=idProduct=4002
udevadm settle --timeout=10
echo "[ker-usb] Installed ${KER_RULE} for ${KER_USER}."
echo "[ker-usb] Restart KER simulation. If access is still denied, reconnect the KER USB cable."
