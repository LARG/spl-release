
alias motion='cd ~/nao/trunk/core/motion/'
alias vision='cd ~/nao/trunk/core/vision/'
alias memory='cd ~/nao/trunk/core/memory/'
alias common='cd ~/nao/trunk/core/common/'
alias naopy='cd ~/nao/trunk/core/python/'
alias core='cd ~/nao/trunk/core/'
alias walk='cd ~/nao/trunk/core/motion/rswalk2014/'
alias build='cd ~/nao/trunk/build'
alias interface='cd ~/nao/trunk/interfaces'

compile() {
  curr=`pwd`
  cd ~/nao/trunk/build
  ./compile "$@"
  cd $curr
}

copy() {
  curr=`pwd`
  cd ~/nao/trunk/build
  ./copy_robot "$@"
  cd $curr
}

runtool() {
  curr=`pwd`
  cd ~/nao/trunk/bin
  ./tool
  cd $curr

}
