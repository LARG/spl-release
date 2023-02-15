cd /boot
mv System.map System.map.bak
mv bzImage bzImage.bak
mv config config.bak
mv vmlinuz vmlinuz.bak
mv vmlinuz-4.4.86-rt99-aldebaran vmlinuz-4.4.86-rt99-aldebaran.bak
mv vmlinuz.efi vmlinuz.efi.bak

cd /home/nao
cp -r lib/modules /lib/
cp boot/* /boot/

cd /boot
ln -s System.map.new System.map
ln -s bzImagenew bzImage
ln -s bzImagenew vmlinuz
ln -s bzImagenew vmlinuz-4.4.86-rt99-aldebaran
ln -s bzImagenew vmlinuz.efi

cd /lib/modules
ln -s 4.4.86-rt99-aldebaran 4.4.86-rt99-aldebaran-g085ca88f9bd1

rm /lib/modules/4.4.86-rt99-aldebaran/4.4.86-rt99-aldebaran
depmod
