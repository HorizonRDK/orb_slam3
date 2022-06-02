#!/bin/bash
TOOL_CHAIN_CMD=
ENABLE_VIEWER=ON
for arg in "$@"
do
  if [ $arg == "x3" ]; then
    AARCH_CONFIG_FILE=`realpath ../../../../robot_dev_config/aarch64_toolchainfile.cmake`
    echo "aarch64_toolchainfile: ${AARCH_CONFIG_FILE}"
    TOOL_CHAIN_CMD=" -DCMAKE_TOOLCHAIN_FILE=${AARCH_CONFIG_FILE}"
    ENABLE_VIEWER=OFF
  fi
done
echo "compile toolchain: ${TOOL_CHAIN_CMD}"

mkdir ./lib

echo "Configuring and building Thirdparty/DBoW2 ..."
cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release ${TOOL_CHAIN_CMD}
make -j
cp ../lib/* ../../../lib/

cd ../../g2o
echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release ${TOOL_CHAIN_CMD}
make -j
cp ../lib/* ../../../lib/

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release ${TOOL_CHAIN_CMD}
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_VIEWER=${ENABLE_VIEWER} ${TOOL_CHAIN_CMD}
make -j4
