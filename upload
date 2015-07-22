#! /bin/bash
#set -x
px_uploader_path=$1
firmware_path=$2

usage()
{
    echo "usage: upload.sh [px_upload.py] [firmware]"
    echo "default: upload.sh Tools/px_uploader.py Images/hifly.px4"
}

if [ -z $px_uploader_path ]; then
    px_uploader_path=$PWD/Tools/px_uploader.py
fi

if [ -z $firmware_path ]; then
    firmware_path=$PWD/Images/hifly.px4
fi

if [ ! -e $px_uploader_path ]; then
    echo "no such file"
    usage
    exit -1
fi

if [ ! -e $firmware_path ]; then
    echo "no such file"
    usage
    exit -1
fi

python -V
if [ $? -ne 0 ]; then
    echo "cannot find python."
    exit -1
fi

echo "px_uploader_path=$px_uploader_path"
echo "firmware_path=$firmware_path"

python -u $px_uploader_path \
    --port "/dev/ttyACM*,/dev/serial/by-id/usb-3D_Robotics*,/dev/serial/by-id/pci-3D_Robotics*" \
     $firmware_path

