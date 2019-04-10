cmake_minimum_required(VERSION 2.8.12)

project(iDynTree_InverseVelocityKinematics CXX)

set(libraryname idyntree-inverse-velocity-kinematics)

set(IDYN_TREE_IVK_SOURCES src/InverseVelocityKinematics.cpp)
set(IDYN_TREE_IVK_HEADERS include/iDynTree/InverseVelocityKinematics.h)

set(PRIVATE_IDYN_TREE_IVK_SOURCES src/InverseVelocityKinematicsData.cpp
                                 src/InverseVelocityKinematicsSolver.cpp
                                 src/VelocityConstraint.cpp)
set(PRIVATE_IDYN_TREE_IVK_HEADERS include/private/InverseVelocityKinematicsData.h
                                 include/private/InverseVelocityKinematicsSolver.h
                                 include/private/VelocityConstraint.h)

source_group("Private\\Header Files" FILES ${PRIVATE_IDYN_TREE_IVK_HEADERS})
source_group("Private\\Source Files" FILES ${PRIVATE_IDYN_TREE_IVK_SOURCES})

add_library(${libraryname} ${IDYN_TREE_IVK_HEADERS} ${IDYN_TREE_IVK_SOURCES}
                           ${PRIVATE_IDYN_TREE_IVK_SOURCES} ${PRIVATE_IDYN_TREE_IVK_HEADERS})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")
target_include_directories(${libraryname} PRIVATE include/private)


# Probably SYSTEM is not compatible with cmake 2.8.12, but I think as YARP requires
# CMake 3.0 we can increase the min version
# target_include_directories(${libraryname} SYSTEM PUBLIC ${IPOPT_INCLUDE_DIRS})
# target_include_directories(${libraryname} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${libraryname} idyntree-core idyntree-high-level ${IPOPT_LIBRARIES})

#target_compile_definitions(${libraryname} PRIVATE ${IPOPT_DEFINITIONS})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYN_TREE_IVK_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)


if(IDYNTREE_COMPILE_TESTS)
  add_subdirectory(tests)
endif()
