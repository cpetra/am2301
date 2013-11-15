MOD=am2301

obj-m := $(MOD).o

all:
	ARCH=arm CROSS_COMPILE=${CCPREFIX} $(MAKE) -C ${KERNEL_SRC} M=$(PWD) modules

clean:
	ARCH=arm CROSS_COMPILE=${CCPREFIX} $(MAKE) -C ${KERNEL_SRC} M=$(PWD) clean
