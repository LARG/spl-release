cd $NAO_HOME/install/robot_libs

scp -r $NAO_HOME/install/robot_libs/i386-linux-gnu  nao@$1:/home/nao/lib/
scp -r $NAO_HOME/install/robot_libs/beignet  nao@$1:/home/nao/lib/
scp $NAO_HOME/lib/detect_lib/gpu-setup/beignet.icd nao@$1:/home/nao/

scp $NAO_HOME/lib/detect_lib/gpu-setup/install_sys_libs.sh nao@$1:/home/nao/
