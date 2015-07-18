#! /bin/bash

#set -x
CC=arm-none-eabi-gcc
CROSSDEV_VER_SUPPORTED="4.8.4 4.9.3"
CROSSDEV_VER_FOUND=`$CC -dumpversion`

echo $CROSSDEV_VER_SUPPORTED
echo $CROSSDEV_VER_FOUND

echo $CROSSDEV_VER_SUPPORTED | grep -q $CROSSDEV_VER_FOUND
if [ $? -ne 0 ]; then
    echo "cannot find supported compile tool"
    exit -1
fi

make distclean
make archives -j8
make hifly -j8
