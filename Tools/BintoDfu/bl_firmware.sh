#!/bin/bash
#./bl_firmware.sh hifly_bl.bin firmware.bin
set -x

BOOTLOADER_SIZE=0x4000
BOOTLOADER_DIR=$1
FIRMWARE_DIR=$2

PACKED_DIR=bl_firmware.bin

rm -f $PACKED_DIR
rm -f output.dfu

cp $BOOTLOADER_DIR $PACKED_DIR

#Finish Bootloader Padding
SIZE=`wc -c $BOOTLOADER_DIR | awk '{ print $1 }'`
BOOTLOADER_PART_SIZE=`printf "%d" $BOOTLOADER_SIZE`
PAD=`expr $BOOTLOADER_PART_SIZE - $SIZE`
dd if=/dev/zero count=1 bs=$PAD 2> /dev/null | \
tr \\000 \\377 >> $PACKED_DIR

#copy firmware
cat $FIRMWARE_DIR >> $PACKED_DIR

echo -e "\n-Packed Image Size"
wc -c $PACKED_DIR

./BintoDfu
