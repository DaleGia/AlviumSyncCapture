# Installs all of the submodules
git submodule update --init --recursive
cd AlliedVisionAlvium

# Installs Allied Vision VimbaX and Camera Drivers
git clone  git@github.com:DaleGia/AlliedVisionVimbaX.git
cd AlliedVisionVimbaX
tar -xzvf VimbaX_Setup-2024-1-Linux64.tar.gz
sudo cp -r VimbaX_2024-1 /usr/local/lib/VimbaX
rm -r VimbaX_2024-1/
sudo sh /usr/local/lib/VimbaX/cti/Install_GenTL_Path.sh
sudo ldconfig


cd ../../
sudo apt-get install -y g++ cmake xorg-dev libglu1-mesa-dev libstdc++6 libopencv-dev libcfitsio-dev gpsd libgps-dev ntpsec
mkdir build
cd build 
cmake ..
cmake --build .
cd ..

cp ntp.conf /etc/ntpsec/ntp.conf
systemctl restart ntpsec.service
