message() {
	echo -e '\E[1;33m'$1'\E[0m'	
}

compile_dts() {

rm -rf arch/arm/boot/dts/imx6-egf-WID$1.dtb
make ARCH=arm imx6-egf-WID$1.dtb
if [ -z arch/arm/boot/dts/imx6-egf-WID$1.dtb ]; then
	message "DTB missing!"
	exit
fi
cp arch/arm/boot/dts/imx6-egf-WID$1.dtb $2
} 


if [ "$1" = "update" ];
then
     message "SELECTED LIVE IMAGE"
     CFG=imx_v7_egf_update_defconfig
     BUILD_EXT="-live"
else
     CFG=imx_v7_egf_defconfig     
     BUILD_EXT=""
fi

export CROSS_COMPILE=/opt/fsl-imx-x11/4.1.15-1.2.0/sysroots/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-
make ARCH=arm $CFG

BUILDVER=$(cat .config | grep LOCALVERSION | awk -F'"' '{print substr($2,2)}')
OUTPUTDIR=build/$BUILDVER$BUILD_EXT
rm -rf $OUTPUTDIR
mkdir -p $OUTPUTDIR

#mkdir -p build/modules/
#rm -rf build/modules/*

rm -rf arch/arm/boot/zImage

make ARCH=arm -j4 zImage
if [ -z arch/arm/boot/zImage ]; then
	message "zImage missing!"
	exit
fi

compile_dts 0510_AA01.01 $OUTPUTDIR
compile_dts 0510_AB01.01 $OUTPUTDIR
compile_dts 0510_AC01.01 $OUTPUTDIR
compile_dts 0510_AC01.02 $OUTPUTDIR
compile_dts 0510_AD01.01 $OUTPUTDIR
compile_dts 0510_AE01.01 $OUTPUTDIR
compile_dts 0510_AE01.02 $OUTPUTDIR
compile_dts 0510_AF01.01 $OUTPUTDIR
compile_dts 0510_AF01.02 $OUTPUTDIR
compile_dts 0510_AG01.01 $OUTPUTDIR
compile_dts 0510_AG01.02 $OUTPUTDIR
compile_dts 0510_AJ01.01 $OUTPUTDIR
compile_dts 0510_AJ01.02 $OUTPUTDIR
compile_dts 0510_AK01.01 $OUTPUTDIR
compile_dts 0510_AK01.02 $OUTPUTDIR

cp arch/arm/boot/zImage $OUTPUTDIR

if [ ! "$1" = "update" ]; then
	make ARCH=arm -j4 modules INSTALL_MOD_PATH=./build/modules
	make ARCH=arm -j4 modules_install INSTALL_MOD_PATH=./build/modules

	mkdir -p $OUTPUTDIR/modules_$BUILDVER
	rm -rf $OUTPUTDIR/modules_$BUILDVER/*
	find build/modules/ -name "*.ko" -exec cp {} $OUTPUTDIR/modules_$BUILDVER/ \;
	tar -C $OUTPUTDIR/modules_$BUILDVER -czvf $OUTPUTDIR/modules_$BUILDVER.tgz  .
fi

message "Output folder: $OUTPUTDIR"
ls -la $OUTPUTDIR
