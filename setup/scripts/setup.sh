#!/usr/bin/env bash

# This script will auto clone all dependencies 
# listed in this packages setup/melodic.rosinstall file.
# You will still need to compile the packages manually.

return_dir=$(pwd)

roscd cloud_segmentor

if [ ! -f "setup/melodic.rosinstall" ]; then
    echo "Something is wrong! I can't find the rosinstall file"
    cd $return_dir
    unset return_dir
    exit 0
fi

rosinstall_file="$(pwd)/setup/melodic.rosinstall"

# Get all the package names
packages=`cat $rosinstall_file | grep local-name: | sed 's/local-name: //g'`

roscd
cd ..

if [ ! -f "src/.rosinstall" ]; then
    echo "Something is wrong! I can't find .rosinstall"
    cd $return_dir
    unset return_dir rosinstall_file packages
    exit 0
fi

cd src

echo "$(pwd)"

echo "-------------------------------------"
echo "wstool merge $rosinstall_file"
echo "-------------------------------------"
wstool merge $rosinstall_file

echo "-------------------------------------"
echo -e "wstool up \n$packages"
echo "-------------------------------------"
wstool up $packages

echo "-------------------------------------"
echo "To finish setup, you'll need to run "
echo "'catkin build' and re-source the "
echo "~/.bashrc"
echo "-------------------------------------"

# Cleanup
cd $return_dir
unset return_dir rosinstall_file packages
