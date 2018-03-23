cmake_minimum_required(VERSION 3.2)
project(TouchDetector)

set(CMAKE_CXX_STANDARD 11)  # for c++11

include(${PROJECT_SOURCE_DIR}/Inference/cmake/FindGlog.cmake)
include(${PROJECT_SOURCE_DIR}/Inference/cmake/FindCaffe.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/freenect2Config.cmake)

find_package(OpenCV 3 REQUIRED core highgui imgproc videoio calib3d)
find_package(Boost REQUIRED filesystem)
link_libraries(${Boost_LIBRARIES})

find_package(Protobuf REQUIRED)
find_package(CUDA REQUIRED)

#link_directories("/home/cz/Downloads/OpenNI-Linux-x64-2.3/Redist")
include_directories(${freenect2_INCLUDE_DIRS})

link_directories(${PROJECT_SOURCE_DIR}/3rdparty/dlib/build/dlib)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/dlib)

# !!IMPORTANT!! Manual setting of linking order to get OpenNI2 to link
set(LINK_LIBS /usr/local/lib/libopencv_core.so.3.2
              /usr/local/lib/libopencv_highgui.so.3.2
              /usr/local/lib/libopencv_imgcodecs.so.3.2
	      /usr/local/lib/libopencv_videoio.so.3.2
              /usr/local/lib/libopencv_imgproc.so.3.2
              /usr/local/lib/libopencv_calib3d.so.3.2
              pthread
              dlib)

include_directories(Inference/include)
add_library(Inference Inference/src/preprocess.cpp Inference/src/inference.cpp)
target_link_libraries(Inference ${Caffe_LIBRARIES} ${GLOG_LIBRARY})

include_directories(RDF/include)
cuda_add_library(cu_kernel RDF/src/rdf/RDF_CU.cu RDF/include/rdf/RDF_CU.cuh OPTIONS -arch sm_35)
add_library(CUDA_RDF RDF/include/proto/rdf.pb.cc
                     RDF/src/rdf/depthImage.cpp
                     RDF/src/rdf/rgbImage.cpp
                     RDF/src/rdf/forest.cpp
                     RDF/src/rdf/sample.cpp
                     RDF/src/rdf/aggregator.cpp
                     RDF/src/rdf/node.cpp
                     RDF/src/rdf/target.cpp
                     RDF/src/rdf/tree.cpp
                     RDF/src/rdf/feature.cpp
                     RDF/src/rdf/logSpace.cpp
                 )
target_link_libraries(CUDA_RDF ${PROTOBUF_LIBRARIES} cu_kernel)

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIRS})

add_executable(touchDetector main.cpp
                             websocket_server.cpp
                             TouchDetection.cpp
                             HandPoseEstimator.cpp
                             RandomForest.cpp
                             DLibInterface.cpp
			     HeatmapInterpreter.cpp
			     HandModel.cpp
			     HandPSO.cpp
			     Fuser.cpp
			     BoundingBox.cpp)
target_link_libraries(touchDetector ${LINK_LIBS} ${CURSES_LIBRARIES} Inference CUDA_RDF ${freenect2_LIBRARIES})

add_executable(calibrateProCam calibrate_pro_cam.cpp)
target_link_libraries(calibrateProCam ${LINK_LIBS})
