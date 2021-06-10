cmake_minimum_required(VERSION 3.12)

project(ForceDimensions VERSION 0.1)

find_path(FORCEDIMENSIONS_SDK_PATH "force-dimensions") 

add_library(dhd SHARED IMPORTED)
set_target_properties(dhd PROPERTIES
  IMPORTED_LOCATION "${FORCEDIMENSIONS_SDK_PATH}/lib/release/lin-x86_64-gcc/libdhd.so.3.11.1"
  INTERFACE_INCLUDE_DIRECTORIES "${FORCEDIMENSIONS_SDK_PATH}/include"
)


sofa_find_package(SofaHaptics REQUIRED)

set(FORCEDIMENSIONS_SRC_DIR src/ForceDimensions)

set(HEADER_FILES
  ${FORCEDIMENSIONS_SRC_DIR}/initForceDimensions.h
  ${FORCEDIMENSIONS_SRC_DIR}/OmegaDriver.h)

set(SOURCE_FILES
  ${FORCEDIMENSIONS_SRC_DIR}/initForceDimensions.cpp
  ${FORCEDIMENSIONS_SRC_DIR}/OmegaDriver.cpp)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} SofaHaptics dhd)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_FORCEDIMENSIONS")

## Installation rules for the library; CMake package configurations files
sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR ${FORCEDIMENSIONS_SRC_DIR}
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )
