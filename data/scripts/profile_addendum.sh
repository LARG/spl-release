#!/usr/bin/env bash

export LD_LIBRARY_PATH=/home/nao/bin:/home/nao/caffe-32/lib:/home/nao/caffe-32/caffe/lib
export PYTHONPATH=/lib/python2.7:/usr/lib/python2.7:/home/nao/bin
export PYTHONHOME=/usr
alias clear='echo -en "\ec"'
alias vim='LD_LIBRARY_PATH=/home/nao/vim/lib vim'
export GLOG_minloglevel=2

alias start='/home/nao/bin/villa-start.sh'
alias stop='/home/nao/bin/villa-stop.sh'
alias nao='/home/nao/bin/shorthand.sh'
