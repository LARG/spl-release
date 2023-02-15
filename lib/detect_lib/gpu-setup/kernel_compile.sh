# Setup toolchain
ln -s $HOME/nao/trunk/naoqi .
./naoqi/crosstoolchain/atom/yocto-sdk/relocate_qitoolchain.sh
cp "$HOME/nao/trunk/lib/detect_lib/gpu-setup/.config" .config

# Run menuconfig to allow to confirm that i915 is enabled in device drivers -> graphics support -> intel 8xx/9xx/G3x/G4x/HDGraphics in kernel module form
make ARCH=x86_64 CROSS_COMPILE=naoqi/crosstoolchain/atom/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/libexec/i686-sbr-linux/gcc/i686-sbr-linux/5.3.0/ -j4 menuconfig

# Compile kernel with nao toolchain
VERBOSE=1 make ARCH=x86_64 CROSS_COMPILE=naoqi/crosstoolchain/atom/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/libexec/i686-sbr-linux/gcc/i686-sbr-linux/5.3.0/ -j8

# Compile modules
VERBOSE=1 make ARCH=x86_64 CROSS_COMPILE=naoqi/crosstoolchain/atom/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/libexec/i686-sbr-linux/gcc/i686-sbr-linux/5.3.0/ -j8 modules
