export PATH="$NAO_HOME/install/cmake-3.22.2-linux-x86_64/bin:$PATH"

cmake --build "`pwd`" -j8 -v --target clean
