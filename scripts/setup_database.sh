#!/usr/bin/env bash
cd "$(cd -P -- "$(dirname -- "$0")" && pwd -P)"

set -e

function reload_daemon() {
  echo "Reloading daemons"
  sudo systemctl daemon-reload
  sudo systemctl restart local-fs.target
}

reload_daemon

echo "Installing LVM2"
sudo apt install lvm2

echo "Backing up fstab to /etc/fstab.bak"
sudo cp /etc/fstab /etc/fstab.bak

echo "Creating mount points /mnt/master /mnt/slave1 /mnt/slave2"
sudo mkdir -p /mnt/master /mnt/slave1 /mnt/slave2

echo "Adding FSTAB entries"
sudo sh -c 'echo "# /dev/mapper/db0-data

UUID=ebc4c166-eab5-49a1-803a-d8735731b6af /mnt/master       ext4       rw,noauto,relatime,x-systemd.automount,x-systemd.idle-timeout=1min 0 2

# /dev/mapper/db1-data

UUID=f522457d-da50-46f0-90cb-6ea64a548bc7 /mnt/slave1       ext4       rw,noauto,relatime,x-systemd.automount,x-systemd.idle-timeout=1min 0 2

# /dev/mapper/db2-data

UUID=f87d9fba-4c40-4e62-b401-3df0b5c01740 /mnt/slave2       ext4       rw,noauto,relatime,x-systemd.automount,x-systemd.idle-timeout=1min 0 2" >> /etc/fstab'

reload_daemon
