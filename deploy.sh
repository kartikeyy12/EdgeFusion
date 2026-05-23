#!/bin/bash
# Usage: ./deploy.sh <JETSON_IP>
# Example: ./deploy.sh 192.168.1.45
JETSON_IP=$1
JETSON_USER="nvidia"

if [ -z "$JETSON_IP" ]; then
    echo "Usage: ./deploy.sh <JETSON_IP>"
    exit 1
fi

echo "=== Building for ARM64 ==="
cmake -S . -B build-arm64 \
    -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release
cmake --build build-arm64 -j$(nproc)

echo "=== Deploying to Jetson at $JETSON_IP ==="
ssh ${JETSON_USER}@${JETSON_IP} "mkdir -p ~/telemetry_pipeline/models"

# Copy binary
scp build-arm64/telemetry_pipeline \
    ${JETSON_USER}@${JETSON_IP}:~/telemetry_pipeline/

# Copy YOLO models
scp models/yolov4-tiny.cfg \
    models/yolov4-tiny.weights \
    models/coco.names \
    ${JETSON_USER}@${JETSON_IP}:~/telemetry_pipeline/models/

# Copy setup script
scp jetson_setup.sh \
    ${JETSON_USER}@${JETSON_IP}:~/telemetry_pipeline/

echo "=== Done. Run on Jetson: ==="
echo "ssh ${JETSON_USER}@${JETSON_IP}"
echo "cd ~/telemetry_pipeline && ./telemetry_pipeline"
