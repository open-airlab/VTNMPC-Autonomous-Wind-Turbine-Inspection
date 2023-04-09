# - Config file for the OpticalFlow package
# It defines the following variables
#  OpticalFlow_INCLUDE_DIRS - include directories
#  OpticalFlow_LIBRARIES    - libraries to link against
 
set(OpticalFlow_INCLUDE_DIRS "/usr/local/include")
#set(OpticalFlow_LIBRARY_DIR "/usr/local/lib")
FIND_LIBRARY(OpticalFlow_LIBRARIES OpticalFlow PATHS "/usr/local/lib" NO_DEFAULT_PATH)
