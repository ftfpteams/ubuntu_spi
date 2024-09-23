build kernel driver:

Step 1:
get Linux kernel source code ,if you have kernel source code,ignore this.

open Software&Update->Ubuntu software,enable "Source code" label

```bash
uname -a
Linux test-KVADRA-LE14U 6.5.13 #2 SMP PREEMPT_DYNAMIC Mon Jun 17 15:45:27 CST 2024 x86_64 x86_64 x86_64 GNU/Linux
```

```bash
#22.04
sudo apt-get source linux-source-6.5.0

#24.04
sudo chown _apt /var/lib/update-notifier/package-data-downloads/partial/
sudo chown -Rv _apt:root /var/lib/apt/lists
sudo apt-get source linux-source-6.8.0
```

install  binutils toolchain

```bash
sudo apt-get update
sudo apt-get install libncurses5-dev 
sudo apt-get install build-essential openssl
sudo apt-get install flex
sudo apt-get install bison
sudo apt-get install openssl
sudo apt-get install libssl-dev
sudo apt-get install libelf-dev
```

Step 2:
copy focal_spi.c to /usr/src/linux-source-6.5.0/drivers/input/touchscreen/

```bash
cd /usr/src/linux-source-6.5.0/drivers/input/touchscreen/
```

change Makefile,add nearby "obj-$(CONFIG_TOUCHSCREEN_SURFACE3_SPI)  += surface3_spi.o

```bash
obj-$(CONFIG_TOUCHSCREEN_FOCAL_SPI)     += focal_spi.o
```

change Kconfig,add nearby "config TOUCHSCREEN_SURFACE3_SPI"

```bash
config TOUCHSCREEN_FOCAL_SPI
           tristate "Focaltech fingerprint SPI "
        depends on SPI
        depends on GPIOLIB || COMPILE_TEST
        help
          Say Y here if you have the Focaltech SPI fingerprint
          controller chip as found on the PC in your system.

          If unsure, say N.

          To compile this driver as a module, choose M here: the
          module will be called focal_spi.
```

Step 3:
build and install kernel

choice Focaltech fingerprint SPI  (TOUCHSCREEN_FOCAL_SPI) [N/m/y/?] (NEW)  m

```bash
#22.04
cd /usr/src/linux-source-6.5.0
sudo cp /boot/config-6.5.0-35-generic .config
sudo make -j10
sudo make modules 
sudo make modules_install
sudo make install
```

```bash
#24.04
sudo chown root:root linux-source-6.8.0/ -R
sudo chmod 777 linux-source-6.8.0/ -R
cd /usr/src/linux-source-6.8.0
sudo cp /boot/config-6.8.0-31-generic .config
sudo make -j10
sudo make modules 
sudo make modules_install
sudo make install
```

restart,then you can see a device as "/dev/focal_moh_spi"

Install deb 

```bash
#22.04
dpkg -i libfprint-2-2_1.94.4+tod1-0ubuntu1~22.04.2_spi_amd64_20240620.deb

#24.04
dpkg -i --force-overwrite libfprint-2-2_1.94.4+tod1-0ubuntu1~22.04.2_spi_amd64_20240620.deb
```



Notice:

```c
/*if spi transfer err,change line 493: spi->mode = SPI_MODE_0*/
	spi->mode = SPI_MODE_0|SPI_CS_HIGH;
```

