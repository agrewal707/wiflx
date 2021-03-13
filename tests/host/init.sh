#!/bin/sh

mknod /dev/net/tap c 10 200
chmod 666 /dev/net/tap
export LD_LIBRARY_PATH=/home/agrewal/local/x86/lib
export PATH=/home/agrewal/local/x86/bin:$PATH
export SOAPY_SDR_ROOT=/home/agrewal/local/x86

