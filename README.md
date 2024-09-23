build kernel driver using dkms:

### Step 1: Dependencies

```bash
sudo apt-get update
sudo apt-get install build-essential dkms
```

## Step 2: Install via dkms

```
sudo dpms add .
sudo dkms add .
sudo dkms build focal_spi/1.0
sudo dkms install focal_spi/1.0
```

## Step 3: Verify it worked

```
sudo dmesg | grep focal
...
ls -l /dev/focal_moh_spi
```

## Step 4: Install modified libfprint

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

