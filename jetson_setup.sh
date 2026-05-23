#!/bin/bash
set -e

echo "=== Updating package lists ==="
sudo apt update

echo "=== Installing build tools ==="
sudo apt install -y cmake git build-essential

echo "=== Installing GStreamer ==="
sudo apt install -y \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-libav \
  gstreamer1.0-tools

echo "=== Installing OpenCV ==="
sudo apt install -y libopencv-dev

echo "=== Installing ZeroMQ ==="
sudo apt install -y libzmq3-dev

echo "=== Installing Eigen3 ==="
sudo apt install -y libeigen3-dev

echo "=== All dependencies installed successfully ==="
