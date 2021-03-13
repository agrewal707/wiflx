#!/bin/bash

NS=$1
DEV=$2
IP=$3
APP=$4
CFG=$5
LOG=$6

ip netns add $NS
$APP $CFG $LOG&
sleep 5
ip link set dev $DEV netns $NS
ip netns exec $NS ip addr add $IP dev $DEV
ip netns exec $NS ip link set dev $DEV up

echo "PRESS CTL-C to exit"
read -r -d ''

killall $APP
