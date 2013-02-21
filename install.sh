#! /bin/bash

# ask for the administrator password upfront
sudo -v

function installDeps() {
	sudo apt-get install git-core cmake freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev
}

function installFreenect() {
	# download freenect and install
	git clone git://github.com/jimjibone/libfreenect.git
	cd libfreenect
	mkdir build
	cd build
	cmake ..
	make
	sudo make install
	sudo ldconfig /usr/local/lib64/
	#sudo glview
	
	# To allow Kinect use when not root
	sudo adduser $USER video
	
	cd ../..
	
	# Make linux device manager rules
	sudo cp -f robotarm/51-kinect.rules /etc/udev/rules.d/51-kinect.rules
	
	echo "Install and build finished. Test libfreenect with \"freenect-glview\"."
}

function installCV() {
	# install prerequisites! LOTS!
	sudo apt-get install libpng12-0 libpng12-dev libpng++-dev libpng3
	sudo apt-get install libpnglite-dev libpngwriter0-dev libpngwriter0c2
	sudo apt-get install zliblg-dbg zliblg zliblg-dev
	sudo apt-get install libjasper-dev libjasper-runtime libjasper1
	sudo apt-get install pngtools libtiff4-dev libtiff4 libtiffxx0c2 libtiff-tools
	sudo apt-get install libjpeg62 libjpeg62-dev libjpeg62-dbg libjpeg-progs
	sudo apt-get install ffmpeg libavcodec-dev libavcodec52 libavformat52 libavformat-dev
	sudo apt-get install libgstreamer0.10-0-dbg libgstreamer0.10-0 libgstreamer0.10-dev
	sudo apt-get install libxinel-ffmpeg libxine-dev libxinel-bin
	sudo apt-get install libunicap2 libunicap2-dev
	sudo apt-get install libdc1394-22-dev libdc1394-22 libdc1394-utils
	sudo apt-get install swig
	sudo apt-get install libv41-0 libv41-dev
	sudo apt-get install python-numpy
	sudo apt-get install libtbb2 libtbb-dev
	
	# download and install opencv
	Start from CV-4
	git clone git://github.com/Itseez/opencv.git
	cd opencv
	mkdir build
	cd build
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
	make
	sudo make install
	
	# to test opencv
	#cd bin
	#./opencv_test_core
	#cd ..
	
	cd ../..
}

function installPCL() {
	# install PCL. super easy
	sudo apt-get-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
	sudo apt-get update
	sudo apt-get install libpcl-all
}

# check that we're running Ubuntu and install
if [[ `uname` == "Linux" ]]; then
	cd ~
	installDeps
	installFreenect
	installCV
	installPCL
	cd ~/robotarm
else
	echo "You're trying to run this on something that isn't Linux. Aborting..."
fi
