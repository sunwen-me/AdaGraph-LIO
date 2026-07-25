#!/usr/bin/env bash

set -euo pipefail

readonly VENDOR_ID="ffff"
readonly PRODUCT_ID="ffff"
readonly SERIAL_NUMBER="2025021200"
readonly DEFAULT_ALIAS="dm_imu"
readonly RULE_FILE="/etc/udev/rules.d/99-dm-imu.rules"

alias_name="${1:-${DEFAULT_ALIAS}}"
alias_name="${alias_name#/dev/}"

if [[ ! "${alias_name}" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "错误：设备别名只能包含字母、数字、点、下划线和连字符。" >&2
  exit 2
fi

if ! command -v udevadm >/dev/null 2>&1; then
  echo "错误：未找到 udevadm。" >&2
  exit 1
fi

sudo_cmd=()
if (( EUID != 0 )); then
  if ! command -v sudo >/dev/null 2>&1; then
    echo "错误：安装 udev 规则需要 root 权限或 sudo。" >&2
    exit 1
  fi
  sudo_cmd=(sudo)
fi

rule="SUBSYSTEM==\"tty\", KERNEL==\"ttyACM[0-9]*\", ATTRS{idVendor}==\"${VENDOR_ID}\", ATTRS{idProduct}==\"${PRODUCT_ID}\", ATTRS{serial}==\"${SERIAL_NUMBER}\", SYMLINK+=\"${alias_name}\", GROUP=\"dialout\", MODE=\"0660\""

tmp_rule="$(mktemp --tmpdir dm-imu-udev.XXXXXX.rules)"
trap 'rm -f "${tmp_rule}"' EXIT
printf '%s\n' "${rule}" > "${tmp_rule}"

echo "正在安装规则：${RULE_FILE}"
"${sudo_cmd[@]}" install -m 0644 "${tmp_rule}" "${RULE_FILE}"
"${sudo_cmd[@]}" udevadm control --reload-rules

device=""
shopt -s nullglob
for candidate in /dev/ttyACM*; do
  if udevadm info --query=property --name="${candidate}" |
      grep -qx "ID_SERIAL_SHORT=${SERIAL_NUMBER}"; then
    device="${candidate}"
    break
  fi
done
shopt -u nullglob

if [[ -n "${device}" ]]; then
  device_path="$(udevadm info --query=path --name="${device}")"
  "${sudo_cmd[@]}" udevadm trigger --action=add --settle "${device_path}"
fi

if [[ -L "/dev/${alias_name}" ]]; then
  echo "完成：/dev/${alias_name} -> $(readlink -f "/dev/${alias_name}")"
else
  echo "规则已安装。请重新插拔 IMU，然后检查 /dev/${alias_name}。"
fi

