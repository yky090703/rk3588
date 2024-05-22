# TARGET_SOC="rk3588"

rm -r build/*

mkdir build

cd build

cmake ..

make -j8

# make install
