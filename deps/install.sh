#! /bin/bash

# ask for the administrator password upfront
sudo -v

PREV=`pwd`

function installFreenectLinux() {
	cd ~
	# get the deps!
	sudo apt-get install git-core cmake freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev
	
	# download freenect and install
	git clone https://github.com/jimjibone/libfreenect.git
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
	sudo cp -f robotarm/deps/51-kinect.rules /etc/udev/rules.d/51-kinect.rules
	cd ~
}

function installCVLinux() {
	cd ~
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
	cd ~
}

function installPCLLinux() {
	# install PCL. super easy
	sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
	sudo apt-get update
	sudo apt-get install libpcl-all
}

function installFreenectMac() {
	# get jimjibone's version of libfreenect as it works
	# better with PCL
	cd ~
	git clone https://github.com/jimjibone/libfreenect.git
	cd libfreenect
	mkdir build
	cd build
	cmake ..
	make
	sudo make install
	cd $PREV
}

function installCVMac() {
	brew install opencv
}

function installPCLMac() {
	# homebrew just fails to download pcl :( this is very sad
	
	# try this way instead
	# ask the user to download PCL manually, then move the file and do brewing
	if [ -e ~/Downloads/PCL* ]; then
		# there is a version somewhere.
		THELOC=`ls ~/Downloads/PCL*`
		
		# where are we? copy the brew file and get going.
		if [ `basename $PWD` == "deps" ]; then
			cp pcl.rb pcl.rb.temp
			sed -i -e 's/auser/'`whoami`'/g' pcl.rb.temp
			sed -i -e 's/afile/'`basename $THELOC`'/g' pcl.rb.temp
			mv pcl.rb.temp /usr/local/Library/Formula/pcl.rb
			brew install pcl
			rm pcl.rb.temp-e
		elif [ `basename $PWD` == "robotarm" ]; then
			cp deps/pcl.rb deps/pcl.rb.temp
			sed -i -e 's/auser/'`whoami`'/g' pcl.rb.temp
			sed -i -e 's/afile/'`basename $THELOC`'/g' pcl.rb.temp
			mv deps/pcl.rb.temp /usr/local/Library/Formula/pcl.rb
			brew install pcl
			rm deps/pcl.rb.temp-e
		else
			echo -e "\x1B[00;31mOh dear.\x1B[00m You need to run this command from at least inside the robotarm directory."
		fi
	else
		echo -e "\x1B[00;31mOh no! You need to download PCL manually first!\n\x1B[00m Sorry. Try pointclouds.org/downloads/ and keep it in your ~/Downloads folder (or root /). Run this script again and I'll do the rest."
	fi
	cd $PREV
}

# check that we're running Ubuntu and install
if [[ `uname` == "Linux" ]]; then
	installFreenectLinux
	installCVLinux
	installPCLLinux
	
# or mac os x
elif [[ `uname` == "Darwin" ]]; then
	echo "You're going to want to accept the Java install if it appears by the way."
	# check for homebrew
	brew update || exit 2
	installFreenectMac
	installCVMac
	installPCLMac
	
# or if you fail
else
	echo "You're trying to run this on something that isn't Linux. Aborting..."
fi
cd $PREV
