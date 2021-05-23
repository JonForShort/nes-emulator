#!/bin/bash

echo ""
echo "----------------------------------------"
echo "Before Freeing Disk Space"
echo "----------------------------------------"
free
df -h
echo "----------------------------------------"

# Remove Android SDK
sudo rm -rf /usr/local/lib/android

# Remove DotNet
sudo rm -rf /usr/share/dotnet

# Remove Haskell
sudo rm -rf /opt/ghc

# Turn off swap
sudo swapoff -a

# Remove swap file
sudo rm -f /mnt/swapfile

# Delete unused docker images
docker system prune -fa

echo ""
echo "----------------------------------------"
echo "After Freeing Disk Space"
echo "----------------------------------------"
free
df -h
echo "----------------------------------------"

