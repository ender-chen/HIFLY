#! /bin/bash
set -x
bl_path=$1
build_sn=$2
top_dir=$PWD

git_hash=`git log -1 --pretty=format:%H | cut -c1-6`
build_date=`date '+%y%m%d'`

usage()
{
    echo "usage: ./package.sh bl_path build_sn"
}

if [ -z $bl_path ]; then
	echo "paramter error"
	usage
	exit -1
fi

if [ -z $build_sn ]; then
	echo "paramter error"
	usage
	exit -1
fi

if [ ! -e $bl_path ]; then
    echo "cannot find bootloader"
	exit -1
fi

bin2dfu_dir=$top_dir/Tools/BintoDfu
bin2dfu_path=$bin2dfu_dir/BintoDfu
cd $bin2dfu_dir && make -B
if [ ! -e $bin2dfu_path ]; then
    echo "cannot find BintoDfu"
	exit -1
fi

firmware_path=$top_dir/build_hifly/src/firmware/nuttx/firmware_nuttx.bin
if [ ! -e $firmware_path ]; then
    echo "cannot find firmware binary"
	exit -1
fi

bl_firmware_path=$bin2dfu_dir/bl_firmware.sh
$bl_firmware_path $bl_path $firmware_path

cd $top_dir
firmware_prefix=HIFLY01A-F-S00A_CKT_L1EN
cp $bin2dfu_dir/output.dfu $top_dir/build_hifly/src/firmware/nuttx/$firmware_prefix\_$build_sn\_$build_date\_$git_hash.dfu
