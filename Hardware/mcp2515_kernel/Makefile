# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)
	obj-m := mcp2515_can.o 

# Otherwise we were called directly from the command
# line; invoke the kernel build system.
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD  := $(shell pwd)

default: clean build sign
	
clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

build:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

sign:
	/usr/src/linux-headers-$(shell uname -r)/scripts/sign-file sha256 \
                                module_sign_key/mcp2515_sign_key.priv \
                                module_sign_key/mcp2515_sign_key.der \
				 mcp2515_can.ko

install: default
	cp mcp2515_can.ko /lib/modules/$(shell uname -r)/kernel/drivers/platform/x86
	depmod -A
	
	echo "mcp2515_can" >> /etc/modules

	$(MAKE) clean

uninstall:
	modprobe -r mcp2515_can
	rm -f /lib/modules/$(shell uname -r)/kernel/drivers/platform/x86/mcp2515_can.*
	depmod -A
	
	sed -i '/mcp2515_can/d' /etc/modules

endif

