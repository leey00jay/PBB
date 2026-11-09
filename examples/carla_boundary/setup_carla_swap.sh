#!/usr/bin/env bash
set -euo pipefail

# Run once with sudo. Creates only this named file and backs up fstab.
swap_path=/swapfile-carla
swap_bytes=17179869184
if [[ $EUID -ne 0 ]]; then
  echo 'Run this script with sudo.' >&2
  exit 1
fi
if [[ -e "$swap_path" || -L "$swap_path" ]]; then
  if swapon --show=NAME --noheadings | grep -Fxq "$swap_path"; then
    echo "$swap_path is already active."
  else
    echo "$swap_path already exists but is inactive; refusing to overwrite it." >&2
    exit 1
  fi
else
  available_bytes=$(df --output=avail -B1 / | tail -n 1 | tr -d ' ')
  if (( available_bytes < swap_bytes + 10737418240 )); then
    echo 'Not enough free space for 16 GiB swap plus 10 GiB headroom.' >&2
    exit 1
  fi
  if [[ $(findmnt -n -o FSTYPE /) != ext4 ]]; then
    echo 'Expected the verified ext4 root filesystem; aborting.' >&2
    exit 1
  fi
  umask 077
  (set -o noclobber; : > "$swap_path")
  chmod 600 "$swap_path"
  fallocate -l "$swap_bytes" "$swap_path"
  mkswap "$swap_path"
  swapon "$swap_path"
fi
if ! awk '$1 == "/swapfile-carla" && $3 == "swap" { found=1 } END { exit !found }' /etc/fstab; then
  backup_path=$(mktemp /etc/fstab.before-carla-swap.XXXXXX)
  cp -p /etc/fstab "$backup_path"
  printf '\n# CARLA memory headroom (16 GiB)\n/swapfile-carla none swap sw,nofail 0 0\n' >> /etc/fstab
  echo "fstab backup: $backup_path"
fi
swapon --show
free -h
