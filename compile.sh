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

export CROSS_COMPILE=/opt/0556_evb_pop/sysroots/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-
export KCFLAGS=--sysroot=/opt/0556_evb_pop/sysroots/cortexa9hf-vfp-neon-poky-linux-gnueabi
export KCPPFLAGS=
export KAFLAGS=

if [ "$1" = "update" ];
then
     message "SELECTED LIVE IMAGE"
     CFG=imx_v7_egf_update_defconfig
     BUILD_EXT="-live"
else
     CFG=imx_v7_egf_defconfig     
     BUILD_EXT=""
fi

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

compile_dts 0500_AA01.01 $OUTPUTDIR
compile_dts 0500_AB01.01 $OUTPUTDIR
compile_dts 0500_AC01.01 $OUTPUTDIR
compile_dts 0500_AD01.01 $OUTPUTDIR
compile_dts 0500_AE01.01 $OUTPUTDIR

cp arch/arm/boot/zImage $OUTPUTDIR

if [ ! "$1" = "update" ]; then
	KERNEL_DIR=$(pwd)
	cd compat_wireless
	make ARCH=arm defconfig-wl18xx
	make ARCH=arm KLIB_BUILD=$KERNEL_DIR -j8
	make ARCH=arm KLIB_BUILD=$KERNEL_DIR -j8 modules INSTALL_MOD_PATH=build/modules
	make ARCH=arm KLIB_BUILD=$KERNEL_DIR -j8 modules_install INSTALL_MOD_PATH=build/modules
	cd ..
	mkdir -p $OUTPUTDIR/modules_$BUILDVER
	rm -rf $OUTPUTDIR/modules_$BUILDVER/*
	find build/modules/ -name "*.ko" -exec cp {} $OUTPUTDIR/modules_$BUILDVER/ \;
	tar -C $OUTPUTDIR/modules_$BUILDVER -czvf $OUTPUTDIR/modules_$BUILDVER.tgz  .
fi

message "Output folder: $OUTPUTDIR"
ls -la $OUTPUTDIR
