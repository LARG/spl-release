cd $NAO_HOME/lib/detect_lib/gpu-setup/

scp kernel_preinstall.sh kernel_install.sh nao@$1:/home/nao/

ssh -t nao@$1 'sudo mount -o remount,rw /'

ssh -t nao@$1 'cd /home/nao;bash /home/nao/kernel_preinstall.sh'

cd $NAO_HOME/install/linux_aldebaran
bash $NAO_HOME/lib/detect_lib/gpu-setup/copy_kernel.sh $1

cd $NAO_HOME/lib/detect_lib/gpu-setup
ssh -t nao@$1 'cd /home/nao;sudo bash /home/nao/kernel_install.sh'
