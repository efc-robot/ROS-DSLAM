#!/bin/bash
# first parameter:path to dict of caffe-ssd
# second parameter:path to dict of g2o
# third parameter:path to image data


sed -i "8c SET(CAFFE_DIR /root/caffe-ssd/)" CMakeLists.txt
sed -i "9c SET(G2O_CMAKE_MODULES /root/g2o/cmake_modules/)" CMakeLists.txt
sed -i "14c SET(CAFFE_DIR /root/caffe-ssd/)" superpointlib/CMakeLists.txt
sed -i "16c \ \ char FileDir[200] = \"/root/data/sq_502_503/image/\";" src/publish_merge_node.cpp
sed -i "132c \ \ SuperPoint superpoint = SuperPoint(\"`pwd\\`/superpointlib/model/superpoint.prototxt\", \"`pwd`/superpointlib/model/superpoint.caffemodel\", 200);" src/superpoint_EF_node.cpp
