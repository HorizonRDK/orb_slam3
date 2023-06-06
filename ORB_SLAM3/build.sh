#!/bin/bash
set -e
TOOL_CHAIN_CMD=
ENABLE_VIEWER=ON
FILE_PATH=`pwd`

clean_Thirdparty() {
  echo "clean Thirdparty"
  cd ${FILE_PATH}/Thirdparty
  for file in ./*
  do
    if [ -d "${file}" ]
    then
      echo "clean the ${file}"
      rm -rf ${file}/build ${file}/lib
    fi
  done
}

build_Thirdparty() {
  echo "build Thirdparty"
  cd ${FILE_PATH}/Thirdparty
  for file in ./*
  do
    if [ -d "${file}" ]
    then
      echo "build the ${file}"
      cd ${file}
      rm -rf ./build ./lib
      mkdir build
      cd build
      cmake .. -DCMAKE_BUILD_TYPE=Release ${TOOL_CHAIN_CMD}
      make -j
      if [ -d "../lib" ]
      then
        cp ../lib/* ../../../lib/
      fi
      cd ../..
    fi
  done
}

build_Examples_ROS2(){
  echo "build Examples_ROS2"
  cd ${FILE_PATH}/Examples_ROS2
  colcon build --merge-install  --cmake-force-configure  --cmake-args  -DCMAKE_BUILD_TYPE=Release  -DBUILD_TESTING:BOOL=OFF  ${TOOL_CHAIN_CMD}
  cd ..
  
}


uncompress_vocabulary() {
  echo "Uncompress vocabulary ..."
  cd ${FILE_PATH}/Vocabulary
  tar -xvf ORBvoc.txt.tar.gz
  cd ..
}

for arg in "$@"
do
  if [ $arg == "x3" ]; then
    export TARGET_ARCH=aarch64
    export TARGET_TRIPLE=aarch64-linux-gnu
    export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
    AARCH_CONFIG_FILE=`realpath ../../../../../robot_dev_config/aarch64_toolchainfile.cmake`
    echo "aarch64_toolchainfile: ${AARCH_CONFIG_FILE}"
    TOOL_CHAIN_CMD=" -DCMAKE_TOOLCHAIN_FILE=${AARCH_CONFIG_FILE}"
    ENABLE_VIEWER=OFF
  fi
done
echo "compile toolchain: ${TOOL_CHAIN_CMD}"

#rm -rf ./build ./lib

mkdir -p ./lib

build_Thirdparty

echo "Configuring and building ORB_SLAM3 ..."
cd ${FILE_PATH}
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_VIEWER=${ENABLE_VIEWER} ${TOOL_CHAIN_CMD}
make -j2

#build_Examples_ROS2
#clean_Thirdparty
#uncompress_vocabulary

