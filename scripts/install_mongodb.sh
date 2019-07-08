#!/usr/bin/env bash
cd "$(cd -P -- "$(dirname -- "$0")" && pwd -P)"

set -e

sudo systemctl stop mongod || true
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv 9DA31620334BD75D9DCB49F368818C72E52529D4
echo "Removing old mongodb sources list (please answer yes to all)"
sudo rm -i /etc/apt/sources.list.d/mongodb-org-* || true
echo "Adding new sources list"
echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu xenial/mongodb-org/4.0 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-4.0.list
sudo apt-get update
sudo apt remove mongodb-org mongodb-org-server mongodb-org-shell mongodb-org-mongos mongodb-org-tools --assume-yes || true
sudo apt install mongodb-org=4.0.1 mongodb-org-server=4.0.1 mongodb-org-shell=4.0.1 mongodb-org-mongos=4.0.1 mongodb-org-tools=4.0.1 --assume-yes
sudo systemctl start mongod
