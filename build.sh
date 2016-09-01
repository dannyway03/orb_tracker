# Execute this script before running a catkin_make

echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
rm -r build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 4

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

rm -r build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 4

cd ../../../

echo "Uncompress vocabulary ..."

cd vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..
