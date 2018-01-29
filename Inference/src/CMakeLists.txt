######### Inference Library ######### 
file(GLOB INFERENCE_LIB_SOURCES ./*.cpp  ./*.c)

add_library(InferenceLibrary ${INFERENCE_LIB_SOURCES})



