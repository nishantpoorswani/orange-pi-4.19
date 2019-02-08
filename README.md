<<<<<<< HEAD
Mainline linux kernel for Orange Pi PC/PC2/One
----------------------------------------------

This kernel tree is meant for:

- Orange Pi PC
- Orange Pi PC 2
- Orange Pi One
- TBS A711 Tablet

You can easily port it to other similar H3/H5 based SBCs by modifying the
appropriate board DTS files. Usually these boards are either like Orange Pi One,
or like Orange Pi PC, or they don't have CPUX voltage regulation at all, so it
should be simple.

Features in addition to mainline:

- Thermal regulation (if CPU heats above certain temperature, it will try to cool itself down by reducing CPU frequency)
- HDMI audio support (from Jernej Skrabec)
- Configure on-board micro-switches to perform system power off function
- Wireguard (https://www.wireguard.com/)

You can use this kernel to run a desktop environment on Orange Pi SBCs.

Have fun!


Build instructions
------------------

These are rudimentary instructions and you need to understand what you're doing.
These are just core steps required to build the ATF/u-boot/kernel. Downloading,
verifying, renaming to correct directories is not described or mentioned. You
should be able to infer missing necessary steps yourself for your particular needs.

Get necessary toolchains from:

- https://releases.linaro.org/components/toolchain/binaries/latest/aarch64-linux-gnu/ for 64bit Orange Pi PC2
- https://releases.linaro.org/components/toolchain/binaries/latest/arm-linux-gnueabihf/ for 32bit Orange Pis

Extract toolchains and prepare the environment:

    CWD=`pwd`
    OUT=$CWD/builds
    SRC=$CWD/u-boot
    export PATH="$PATH:$CWD/Toolchains/arm/bin:$CWD/Toolchains/aarch64/bin"

For Orange Pi PC2:

    export CROSS_COMPILE=aarch64-linux-gnu-
    export KBUILD_OUTPUT=$OUT/.tmp/uboot-pc2
    rm -rf "$KBUILD_OUTPUT"
    mkdir -p $KBUILD_OUTPUT $OUT/pc2

Get and build ATF from https://github.com/apritzel/arm-trusted-firmware/tree/allwinner:
(allwinner branch)

    make -C "$CWD/arm-trusted-firmware" PLAT=sun50iw1p1 DEBUG=1 bl31
    cp "$CWD/arm-trusted-firmware/build/sun50iw1p1/debug/bl31.bin" "$KBUILD_OUTPUT"

Build u-boot from https://github.com/megous/u-boot/commits/orange-pi with appropriate
defconfig (orangepi_one_defconfig, orangepi_pc2_defconfig, orangepi_pc_defconfig). This branch already has
all the necessary patches integrated and is configured for quick u-boot/kernel startup.

    make -C u-boot orangepi_pc2_defconfig
    make -C u-boot -j5
    
    cp $KBUILD_OUTPUT/.config $OUT/pc2/uboot.config
    cat $KBUILD_OUTPUT/{spl/sunxi-spl.bin,u-boot.itb} > $OUT/pc2/uboot.bin

Get kernel from this repository and checkout the latest orange-pi-4.19 branch.

Configure kernel by using the prepared configuration for H5 based Orange Pi boards:

    cp linux-4.19-64 .config

Build the kernel:

    export ARCH=arm64
    export CROSS_COMPILE=aarch64-linux-gnu-
    export KBUILD_OUTPUT=$OUT/.tmp/linux-arm64
    mkdir -p $KBUILD_OUTPUT $OUT/pc2

    make -C linux -j5 clean
    make -C linux -j5 Image dtbs

    cp -f $KBUILD_OUTPUT/arch/arm64/boot/Image $OUT/pc2/
    cp -f $KBUILD_OUTPUT/.config $OUT/pc2/linux.config
    cp -f $KBUILD_OUTPUT/arch/arm64/boot/dts/allwinner/sun50i-h5-orangepi-pc2.dtb $OUT/pc2/board.dtb

Configure kernel by using the prepared configuration for H3 based Orange Pi boards:

    cp linux-4.19-32 .config

Build the kernel:

    export ARCH=arm
    export CROSS_COMPILE=arm-linux-gnueabihf-
    export KBUILD_OUTPUT=$OUT/.tmp/linux-arm
    mkdir -p $KBUILD_OUTPUT $OUT/pc

    make -C linux -j5 clean
    make -C linux -j5 zImage dtbs
    
    cp -f $KBUILD_OUTPUT/arch/arm/boot/zImage $OUT/pc/
    cp -f $KBUILD_OUTPUT/.config $OUT/pc/linux.config
    cp -f $KBUILD_OUTPUT/arch/arm/boot/dts/sun8i-h3-orangepi-pc.dtb $OUT/pc/board.dtb
    # Or use sun8i-h3-orangepi-one.dtb for Orange Pi One


Kernel lockup issues
--------------------

*If you're getting lockups on boot or later during thermal regulation,
you're missing an u-boot patch.*

This patch is necessary to run this kernel!

These lockups are caused by improper NKMP clock factors selection
in u-boot for PLL_CPUX. (M divider should not be used. P divider
should be used only for frequencies below 240MHz.)

This patch for u-boot fixes it:

  0001-sunxi-h3-Fix-PLL1-setup-to-never-use-dividers.patch

Kernel side is already fixed in this kernel tree.


Sample configuration
--------------------

- *linux-4.19-32* file contains working configuration of the kernel for Orange Pi PC/One
- *linux-4.19-64* file contains working configuration of the kernel for Orange Pi PC 2
=======
# orange-pi-4.19
Linux kernel Orange-pi-r1
>>>>>>> 235b7a0de165cee831e932170d2768a2f33f052d
