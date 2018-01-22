How to install gcc and openocd successfully?

1. Install git by running:
	sudo apt-get install git-all

2. Clone the repository from Github to your local

3. Go to the folder gcc4mbed

4. Do gcc installation. Run:
	./linux_install

	If the operation system is 64-bit but the 32-bit binary files are going to execute/compile, run:
	sudo dpkg --add-architecture i386
	sudo apt-get update
	sudo apt-get install build-essential g++
	sudo apt-get install libc6:i386 libncurses5:i386 libstdc++6:i386

5. If the installation is failed, go to "../Motion_Tracking_Using_Magnetometer/gcc4mbed/code/MPU9250AHARS" and open the file "makefile" to correct:
	*******************************

	PROJECT         := MPU9250AHRS -->  MPU9250AHARS
	*******************************

6. Now compile the makefile. Run:
 	make

	then go back to gcc4mbed do: 
	./linux_install
	
	again and this time it should work properly and gcc should be installed

7. Now we need to install openocd. First install hidapi. Run: 
	git clone https://github.com/signal11/hidapi.git
	sudo apt-get install aptitude
	sudo aptitude install pkg-config libusb-1.0-0-dev libudev-dev libfox-1.6-dev autotools-dev autoconf automake libtool
	cd hidapi
    	./bootstrap
    	./configure --prefix=/usr
	make
    	sudo make install

8. Install Openocd. Go to the website "https://sourceforge.net/projects/openocd/files/latest/download?source=files" and download the openocd, unzip it by running:

	tar xf openocd-0.9.0.tar.bz2
 
	Run:
	
	apt-get build-dep openocd
	cd openocd-0.9.0/ 
	./configure --enable-aice --enable-amtjtagaccel --enable-armjtagew --enable-cmsis-dap --enable-dummy \
        --enable-ftdi --enable-gw16012 --enable-jlink --enable-jtag_vpi --enable-opendous \
        --enable-openjtag_ftdi --enable-osbdm --enable-legacy-ft2232_libftdi --enable-parport \
        --disable-parport-ppdev --enable-parport-giveio --enable-presto_libftdi --enable-remote-bitbang \
        --enable-rlink --enable-stlink --enable-ti-icdi --enable-ulink --enable-usb-blaster-2 \
        --enable-usb_blaster_libftdi --enable-usbprog --enable-vsllink
    	make
    	sudo make install


----------------------------------------------------------------------------------------------------------------------------------------
9. Now we are good to run the board! First power up the board by connecting it to usb.

10. In one terminal, start openocd by running:
	sudo openocd -f /usr/local/share/openocd/scripts/board/st_nucleo_f4.cfg

11. In ANOTHER terminal, go to "../Motion_Tracking_Using_Magnetometer/gcc4mbed/gcc-arm-none-eabi/bin", run:

	./arm-none-eabi-gdb
	
	You will see something like this in terminal:
	
	(gdb)

12. In gdb, type:

	(gdb) file ../Motion_Tracking_Using_Magnetometer/gcc4mbed/code/MPU9250AHARS/NUCLEO_F401RE/MPU9250AHARS.elf
    (gdb) target remote :3333
    (gdb) load
    (gdb) monitor arm semihosting enable
    (gdb) monitor reset
	(gdb) continue

13. Now, open ONE MORE NEW TERMINAL and do the following (just test to see if you can retrieve info from the sensor): 
	- in the tools directory (you will have this directory once all 1-8 steps are finished correctly) you will find two scripts: 		acm_reset.sh and watch.py
   	- for this software the baud rate is 9600 - important to use correct baud rate
   	in tools.
	Run:
   	sudo ./acm_reset.sh
   	sudo ./watch.py /dev/ttyACM0 9600

14. If everything goes well, you will see something like this:

	reset cdc_acm kernel module after unplugging SEGGER devices
	523760  deg/s
	mx = -326.062531 my = -32.385883 mz = 185.895309  mG
	temperature = 30.099350  C
	q0 = 0.684724
	q1 = 0.016172
	q2 = -0.013028
	q3 = 0.728506
	Yaw, Pitch, Roll: 79.745132 -2.372995 0.181496
	average rate = 666.065247
	ax = 2.014160 ay = 2.563477 az = 994.995117  mg
	gx = 0.045873 gy = -0.024192 gz = 0.115313  deg/s
	mx = -324.497986 my = -24.563194 mz = 190.422913  mG
	temperature = 30.111332  C
	...
---------------------------------------------------------------------------------------------------------------------------------------

Required python packages:

	1. Download pyQtGraph from http://www.pyqtgraph.org/
	2. sudo apt-get install python python-numpy python-opengl python-qt4 libqt4-opengl python-qt4-gl
	3. sudo apt-get install python-pip
	4. pip install pyserial
	
	You also need to install Pygame for the animation of the "stickman"
	https://www.pygame.org/download.shtml	
---------------------------------------------------------------------------------------------------------------------------------------

To enable serial port on bluetooth communication:

	~$ sudo hcitool scan
	Scanning ...
	00:02:C7:7D:F5:17  HC-06
	~$ sudo rfcomm bind /dev/rfcomm0 <Device MAC address> 1
	~$ ls -l /dev/rfcomm0
	crw-rw---- 1 root dialout 216, 0 2008-12-14 23:15 /dev/rfcomm0