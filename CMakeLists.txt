cmake_minimum_required(VERSION 2.8)
project(libmikrokopter)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
# set(CMAKE_CXX_FLAGS "-pedantic -Wno-variadic-macros -Weffc++ -Wno-long-long")
# set(CMAKE_CXX_FLAGS "-Wno-variadic-macros -Weffc++ -Wno-long-long")
set(CMAKE_CXX_FLAGS "-W -Wall -Wno-unused-parameter")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib) 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin) 
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib) 

set(CPACK_GENERATOR "DEB")
set(CPACK_SOURCE_GENERATOR "TGZ")
set(CPACK_PACKAGE_VERSION_MAJOR 0)
set(CPACK_PACKAGE_VERSION_MINOR 1)
set(CPACK_PACKAGE_VERSION_PATCH 0)
set(CPACK_PACKAGE_CONTACT "Dirk Holz <dirk.holz@ieee.org>")
set(CPACK_SOURCE_IGNORE_FILES "${CPACK_SOURCE_IGNORE_FILES};${CMAKE_SOURCE_DIR}/.git;${CMAKE_BINARY_DIR}/*")
set(CPACK_SOURCE_IGNORE_FILES "${CPACK_SOURCE_IGNORE_FILES};${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/*")
set(CPACK_SOURCE_IGNORE_FILES "${CPACK_SOURCE_IGNORE_FILES};${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/*")
set(CPACK_SOURCE_IGNORE_FILES "${CPACK_SOURCE_IGNORE_FILES};${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/*")
include(CPack)

# add custom 'uninstall' for convenience, for undoing the effects of 'sudo make install'
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

find_package(Boost REQUIRED COMPONENTS thread system)
include_directories(${BOOST_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(mikrokopter
  src/common/time.cpp
  src/io/common.cpp
  src/io/console.cpp
  src/io/serial.cpp
  src/protocol/encoding.cpp
  src/protocol/protocol.cpp
  src/kopter.cpp)
target_link_libraries(mikrokopter ${Boost_LIBRARIES} pthread)

add_executable(test_protocol tools/test_protocol.cpp)
target_link_libraries(test_protocol mikrokopter)

add_executable(test_kopter tools/test_kopter.cpp)
target_link_libraries(test_kopter mikrokopter)

# installation targets
install(DIRECTORY include/ DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
install(DIRECTORY lib/ DESTINATION lib)
install(DIRECTORY bin/ DESTINATION bin
  FILE_PERMISSIONS
  OWNER_READ OWNER_EXECUTE OWNER_WRITE
  GROUP_READ GROUP_EXECUTE
  WORLD_READ WORLD_EXECUTE
  DIRECTORY_PERMISSIONS
  OWNER_READ OWNER_EXECUTE OWNER_WRITE
  GROUP_READ GROUP_EXECUTE
  WORLD_READ WORLD_EXECUTE)

