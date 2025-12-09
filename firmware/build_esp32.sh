#!/bin/bash
# Build and Upload ESP32 Firmware

set -e

echo "========================================"
echo "ESP32 Firmware Build & Upload"
echo "========================================"

cd "$(dirname "$0")"

echo ""
echo "Installing dependencies..."
pio pkg install

echo ""
echo "Building firmware..."
pio run --environment esp32doit-devkit-v1

echo ""
echo "Firmware built successfully!"
echo ""
read -p "Upload to ESP32? (y/n) " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Uploading firmware..."
    pio run --environment esp32doit-devkit-v1 --target upload
    echo ""
    echo "Upload complete!"
    echo ""
    read -p "Monitor serial output? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Starting serial monitor (Ctrl+C to exit)..."
        pio device monitor
    fi
fi

echo "Done!"
