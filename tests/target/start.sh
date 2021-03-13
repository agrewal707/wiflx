#!/bin/sh

DEV=$1
IP=$2
APP=$3
CFG=$4
LOG=$5

$APP $CFG $LOG&
sleep 5
ip addr add $IP dev $DEV
ip link set dev $DEV up

echo "PRESS CTL-C to exit"
read -r -d ''

killall $APP
