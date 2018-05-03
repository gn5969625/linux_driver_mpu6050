PWD=$(shell pwd)
KDIR:=/home/kevin/proj/firefly-rk3288-lollipop/kernel
obj-m:= mpu6050.o
all:
	make ARCH=arm CROSS_COMPILE=/home/kevin/proj/firefly-rk3288-lollipop/prebuilts/gcc/linux-x86/arm/arm-eabi-4.6/bin/arm-eabi- -C $(KDIR) M=$(PWD) modules

clean:
	#make -C $(KERN_DIR) M=`pwd` modules clean
	#rm -rf modules.order
	rm modules.order Module.symvers *.mod.c *.o *.ko
