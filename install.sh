git submodule update --init --recursive
cd AlliedVisionAlvium/AlliedVisionVimbaX
bash install.sh
cd ../../
sudo apt-get install -y g++ cmake xorg-dev libglu1-mesa-dev libstdc++6 libopencv-dev libcfitsio-dev gpsd libgps-dev ntpsec
mkdir build
cd build 
cmake ..
cmake --build .
cd ..
cp ntpsec /etc/ntpsec/ntp.conf
systemctl restart ntpsec.service