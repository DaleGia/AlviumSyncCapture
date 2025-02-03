# Installs all of the submodules
git submodule update --init --recursive
cd AlliedVisionAlvium

# Installs Allied Vision VimbaX and Camera Drivers
git clone  git@github.com:DaleGia/AlliedVisionVimbaX.git
cd AlliedVisionVimbaX
tar -xzvf VimbaX_Setup-2024-1-Linux64.tar.gz
sudo cp -r VimbaX_2024-1 /usr/local/lib/VimbaX
cd ..
rm -r AlliedVisionVimbaX
sudo sh /usr/local/lib/VimbaX/cti/Install_GenTL_Path.sh
sudo ldconfig


#Installs required dependencies
sudo apt-get install -y g++ cmake xorg-dev libglu1-mesa-dev libstdc++6 libopencv-dev libcfitsio-dev gpsd libgps-dev ntpsec

#Builds the project
cd ..
mkdir build
cd build 
cmake ..
cmake --build .
sudo cp -r AlviumSyncCapture /usr/local/bin/
cd ..

#Configures NTP sec
sudo cp ntp.conf /etc/ntpsec/ntp.conf
systemctl restart ntpsec.service

#Configures GPSD to work as a time server
sudo cp gpsd /etc/default/gpsd
systemctl restart gpsd.service
