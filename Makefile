obj-m := focal_spi.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules_install

uninstall:
	rm /lib/modules/$(shell uname -r)/extra/focal_spi.ko
	depmod

