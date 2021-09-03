#!/bin/sh

# This script generates the custom root system overlay
# for Adalm Pluto linux system firmware

TOOLCHAIN_PATH=$HOME/opt/Xilinx/SDK/2019.1/gnu/aarch32/lin/gcc-arm-linux-gnueabi
PLUTOSDR_FW_ROOTOVERLAY=$HOME/projects/plutosdr-fw/rootfs-overlay
INSTALL_DIR=$HOME/local/arm
SCRIPT_DIR=`pwd`/`dirname "$0"`
OBJCOPY=${TOOLCHAIN_PATH}/bin/arm-linux-gnueabihf-objcopy

copy_links() {
	orig_dir=$1
	dest_dir=$2
	file_name=$3
	file_links=`find $orig_dir -lname $file_name`
	for file_link in $file_links; do
		cp -a $file_link $dest_dir/`basename $file_link`
	done
}

BIN_FILES="\
	wiflx_ap \
	wiflx_rm \
	wiflx_test\
	"

LIB_FILES="\
	libboost_system.so.1.76.0 \
	libboost_date_time.so.1.76.0 \
	libSoapySDR.so.0.8.1 \
	libad9361.so.0.2 \
	libiio.so.0.22 \
	"

LIB_DIRS="\
	SoapySDR \
	"

cd $SCRIPT_DIR

mkdir -p $PLUTOSDR_FW_ROOTOVERLAY/root/bin
for file in $BIN_FILES; do
	${OBJCOPY} --strip-all $INSTALL_DIR/bin/$file $PLUTOSDR_FW_ROOTOVERLAY/root/bin/$file
done

mkdir -p $PLUTOSDR_FW_ROOTOVERLAY/root/lib
for file in $LIB_FILES; do
	${OBJCOPY} --strip-all $INSTALL_DIR/lib/$file $PLUTOSDR_FW_ROOTOVERLAY/root/lib/$file
	copy_links $INSTALL_DIR/lib/ $PLUTOSDR_FW_ROOTOVERLAY/root/lib $file
done

for dir in $LIB_DIRS; do
	cp -r $INSTALL_DIR/lib/$dir $PLUTOSDR_FW_ROOTOVERLAY/root/lib
done

mkdir -p $PLUTOSDR_FW_ROOTOVERLAY/root/etc
cp tests/target/*.cfg $PLUTOSDR_FW_ROOTOVERLAY/root/etc
cp tests/*.ftr $PLUTOSDR_FW_ROOTOVERLAY/root


