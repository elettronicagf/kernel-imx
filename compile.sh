export CROSS_COMPILE=/opt/fsl-imx-x11/4.14-sumo/sysroots/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-
export KCFLAGS=
export KCPPFLAGS=
export KAFLAGS=

message() {
	echo -e '\E[1;33m'$1'\E[0m'	
}

compile_dts() {
	rm -rf arch/arm/boot/dts/egf-sbc-WID$1.dtb
	make ARCH=arm egf-sbc-WID$1.dtb
	if [ -z arch/arm/boot/dts/egf-sbc-WID$1.dtb ]; then
		message "DTB missing!"
		exit
	fi
	cp arch/arm/boot/dts/egf-sbc-WID$1.dtb $2
} 


if [ "$1" = "update" ]; then
     message "SELECTED LIVE IMAGE"
     CFG=imx_v7_egf_update_defconfig
     BUILD_EXT="-live"
elif [ "$1" = "dt" ]; then
	W=0659_AA01.01
	rm -rf arch/arm/boot/dts/egf-sbc-WID$W.dtb
	make ARCH=arm egf-sbc-WID$W.dtb
	exit
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

make ARCH=arm  -j4 zImage
if [ -z arch/arm/boot/zImage ]; then
	message "zImage missing!"
	exit
fi

compile_dts 0659_AA01.01 $OUTPUTDIR

cp arch/arm/boot/zImage $OUTPUTDIR

if [ ! "$1" = "update" ]; then
	rm -rf ./build/modules
	make ARCH=arm -j4 modules INSTALL_MOD_PATH=./build/modules
	make ARCH=arm -j4 modules_install INSTALL_MOD_PATH=./build/modules

	tar -C ./build/modules -czf $OUTPUTDIR/modules_$BUILDVER.tgz .
fi

message "Output folder: $OUTPUTDIR"
ls -la $OUTPUTDIR
