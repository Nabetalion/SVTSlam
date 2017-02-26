# CMakeの最低バージョンを記述
cmake_minimum_required(VERSION 2.8)
 
# ソリューション名を指定
project(RecMap)

SET(SOURCE
 main.cpp
 SVTSlam.cpp
 ManageFp.cpp
 LoadData.cpp
 EstimateState.cpp
 EstimateMap.cpp
)

SET(HEADERS
 SVTSlam.h
 ManageFp.h
 LoadData.h
 EstimateState.h
 EstimateMap.h
)

# 実行ファイル名とソース指定
add_executable(RecMap ${SOURCE} ${HEADERS})



# OpenCVインストールパスを指定
set(OpenCV_DIR "C:/dev/opencv-3.1.0contrib/opencv-master/build")

# OpenCVのパッケージを探す
find_package(OpenCV REQUIRED)
 
# OpenCVが見つかった場合のみ設定を行う
if(OpenCV_FOUND)
  # インクルードパスを指定
  include_directories(${OpenCV_INCLUDE_DIRS})
 
  # リンクするライブラリ指定
  target_link_libraries(RecMap ${OpenCV_LIBS})
endif(OpenCV_FOUND)

# Eigen
#include_directories("D:/twatanabe8/dev/eigen-3.2.4")
include_directories("c:/dev/eigen-3.2.8")

# pthread
#include_directories("D:/twatanabe8/dev/pthread/include")
#target_link_libraries(RecMap "D:/twatanabe8/dev/pthread/lib/x64/pthreadVC2.lib")
include_directories("c:/dev/pthread/include")
target_link_libraries(RecMap "c:/dev/pthread/lib/x64/pthreadVC2.lib")

