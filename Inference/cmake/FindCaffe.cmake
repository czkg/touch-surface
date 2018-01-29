# provides Caffe_LIBRARIES and Caffe_INCLUDE_DIRS

# Caffe package
unset(Caffe_FOUND)

###Set the variable Caffe_DIR as the root of your caffe directory
set(Caffe_DIR /home/cz/caffe CACHE PATH "Directory to find Caffe includes")


find_path(Caffe_INCLUDE_DIRS NAMES caffe/caffe.hpp caffe/common.hpp caffe/net.hpp caffe/util/io.hpp caffe/layer.hpp
  HINTS
  ${Caffe_DIR}/include)

# TODO make sure $Caffe/build/include exists and has proto/caffe.pb.h

find_library(Caffe_LIBRARIES NAMES caffe
  HINTS
  ${Caffe_DIR}/build/lib)

message(STATUS "Found Caffe lib dirs: ${Caffe_LIBRARIES}")

if(Caffe_LIBRARIES AND Caffe_INCLUDE_DIRS)
    set(Caffe_FOUND 1)
    include_directories(${Caffe_INCLUDE_DIRS})
    include_directories(${Caffe_DIR}/build/include)
else()
    message(FATAL_ERROR "Cannot find Caffe")
endif()
