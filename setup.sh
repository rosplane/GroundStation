#!/bin/bash

# Do we want to check that ground_ws is there?

# Install marble and python bindings
git clone git://anongit.kde.org/marble.git ~/marble/sources -b KDE/4.12
mkdir -p ~/marble/build
cd ~/marble/build
sudo apt-get install kdelibs5-dev
cmake ~/marble/sources -DCMAKE_BUILD_TYPE=Release -DEXPERIMENTAL_PYTHON_BINDINGS=TRUE -DCMAKE_INSTALL_PREFIX=/usr/local
make
sudo make install
sudo apt-get install python-kde4-dev

# Install the geographiclib python package (lets us do NED to lat/long conversions)
sudo apt-get install python-pip
sudo pip install geographiclib

# Download the google maps library
git clone https://github.com/freayd/marble-maps.git ~/.local/share/marble/maps/
