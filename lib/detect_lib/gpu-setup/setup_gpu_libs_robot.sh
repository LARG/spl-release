cd $NAO_HOME/lib/detect_lib/gpu-setup

ssh -t nao@$1 'mkdir -p /home/nao/lib/i386-linux-gnu/'

bash copy_libs.sh $1

ssh -t nao@$1 'cd /home/nao;bash /home/nao/install_sys_libs.sh'

scp ../libdetect_lib.so nao@$1:/home/nao/lib/i386-linux-gnu