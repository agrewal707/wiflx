#!/bin/sh

mknod /dev/net/tap c 10 200
chmod 666 /dev/net/tap

export LD_LIBRARY_PATH=/tmp/host/local/arm/lib
export PATH=/tmp/host/local/arm/bin:$PATH
export SOAPY_SDR_ROOT=/tmp/host/local/arm/
