#!/bin/bash
# Start Micro XRCE-DDS Agent

MODE=${1:-"serial"}
DEVICE=${2:-"/dev/ttyACM0"}
BAUDRATE=${3:-"921600"}
PORT=${3:-"8888"}

echo "============================================"
echo "  Starting Micro XRCE-DDS Agent"
echo "============================================"

if [ "$MODE" == "serial" ]; then
    echo "Mode: Serial"
    echo "Device: $DEVICE"
    echo "Baudrate: $BAUDRATE"
    echo ""
    MicroXRCEAgent serial --dev $DEVICE -b $BAUDRATE
elif [ "$MODE" == "udp" ]; then
    echo "Mode: UDP"
    echo "Port: $PORT"
    echo ""
    MicroXRCEAgent udp4 -p $PORT
else
    echo "Usage: $0 [serial|udp] [device|port] [baudrate]"
    echo ""
    echo "Examples:"
    echo "  $0 serial /dev/ttyUSB0 921600"
    echo "  $0 udp 8888"
    exit 1
fi
