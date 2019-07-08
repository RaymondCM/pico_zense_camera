#!/usr/bin/env bash
cd $(cd -P -- "$(dirname -- "$0")" && pwd -P)

# Install i2c rules

# See https://wiki.odroid.com/troubleshooting/gpiomem
sudo apt-get install i2c-tools python-smbus
sudo addgroup gpio
sudo usermod -a -G gpio odroid
sudo usermod -a -G i2c odroid

# meson-gpiomem is for C1/C2
# exynos-gpiomem is for XU4

echo """SUBSYSTEM==\"exynos-gpiomem\", GROUP=\"gpio\", MODE=\"0660\"
SUBSYSTEM==\"meson-gpiomem\", GROUP=\"gpio\", MODE=\"0660\"
SUBSYSTEM==\"i2c-dev\", ATTR{name}==\"s3c2410-i2c\" , GROUP=\"gpio\", MODE=\"0660\"
""" | sudo tee /etc/udev/rules.d/90-gpiomem.rules

echo """SUBSYSTEM==\"gpio\", KERNEL==\"gpiochip*\", ACTION==\"add\", PROGRAM=\"/bin/sh -c 'chown root:gpio /sys/class/gpio/export /sys/class/gpio/unexport ; chmod 220 /sys/class/gpio/export /sys/class/gpio/unexport'\"
SUBSYSTEM==\"gpio\", KERNEL==\"gpio*\", ACTION==\"add\", PROGRAM=\"/bin/sh -c 'chown root:gpio /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value ; chmod 660 /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value'\"
""" | sudo tee /etc/udev/rules.d/90-odroid-sysfs.rules   

echo -e "KERNEL==\"i2c-[0-7]\",MODE=\"0666\"" | sudo tee /etc/udev/rules.d/17-i2c.rules

# reload UDEV rules
sudo udevadm control --reload
