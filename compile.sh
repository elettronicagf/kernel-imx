export CROSS_COMPILE=/opt/fsl-imx-x11/4.1.15-2.1.0/sysroots/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-
export KCFLAGS=
export KCPPFLAGS=
export KAFLAGS=
export ARCH=arm

message() {
	echo -e '\E[1;33m'$1'\E[0m'	
}

compile_dts() {

rm -rf arch/arm/boot/dts/imx7d-egf-WID$1.dtb
make imx7d-egf-WID$1.dtb
if [ -z arch/arm/boot/dts/imx7d-egf-WID$1.dtb ]; then
	message "DTB missing!"
	exit
fi
cp arch/arm/boot/dts/imx7d-egf-WID$1.dtb $2
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

make $CFG

BUILDVER=$(cat .config | grep LOCALVERSION | awk -F'"' '{print substr($2,1)}')
OUTPUTDIR=build/$BUILDVER$BUILD_EXT
rm -rf $OUTPUTDIR
mkdir -p $OUTPUTDIR

#mkdir -p build/modules/
#rm -rf build/modules/*

rm -rf arch/arm/boot/zImage

make -j4 zImage
if [ -z arch/arm/boot/zImage ]; then
	message "zImage missing!"
	exit
fi

compile_dts 0575_AA01.01 $OUTPUTDIR
compile_dts 0575_AB01.01 $OUTPUTDIR
compile_dts 0575_AA01.02 $OUTPUTDIR

cp arch/arm/boot/zImage $OUTPUTDIR

if [ ! "$1" = "update" ]; then
	make -j4 modules INSTALL_MOD_PATH=./build/modules
	make -j4 modules_install INSTALL_MOD_PATH=./build/modules

	mkdir -p $OUTPUTDIR/modules_$BUILDVER
	rm -rf $OUTPUTDIR/modules_$BUILDVER/*
	find build/modules/ -name "*.ko" -exec cp {} $OUTPUTDIR/modules_$BUILDVER/ \;
	tar -C ./build/modules -czf $OUTPUTDIR/modules_$BUILDVER.tgz  .
fi

message "Output folder: $OUTPUTDIR"
ls -la $OUTPUTDIR
