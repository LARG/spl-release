export PATH="$NAO_HOME/install/cmake-3.22.2-linux-x86_64/bin:$PATH"

export LDFLAGS=-m32

cd build

cmake -DTFLITE_ENABLE_GPU=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-m32 -std=gnu++14" -DCMAKE_C_FLAGS="-m32 -std=gnu++14" "`pwd`/.."

cmake --build "`pwd`" -j8 -v --target test_detect_lib benchmark_model
