echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
export C_INCLUDE_PATH=/usr/local/boost-1.77/include:/usr/local/eigen/include/eigen3:/usr/local/opencv-4.5.3/include/opencv4
export CPLUS_INCLUDE_PATH=/usr/local/boost-1.77/include:/usr/local/eigen/include/eigen3:/usr/local/opencv-4.5.3/include/opencv4
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
