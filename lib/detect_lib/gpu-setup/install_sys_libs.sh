sudo mount -o remount,rw /
sudo mkdir -p /usr/local/lib
sudo cp -r /home/nao/lib/beignet /usr/local/lib/
sudo mkdir -p /etc/OpenCL/vendors/
sudo cp beignet.icd /etc/OpenCL/vendors/beignet.icd
sudo cp /home/nao/lib/i386-linux-gnu/libOpen* /usr/lib/
