cmake_minimum_required(VERSION 2.8)
project(libmikrokopter)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib) 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin) 
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib) 

# note: for some reason CPACK has to be run twice for proper package generation
set(CPACK_GENERATOR "DEB")
set(CPACK_SOURCE_GENERATOR "TGZ")
set(CPACK_PACKAGE_VERSION_MAJOR 0)
set(CPACK_PACKAGE_VERSION_MINOR 1)
set(CPACK_PACKAGE_VERSION_PATCH 0)
set(CPACK_PACKAGE_CONTACT "Dirk Holz <dirk.holz@ieee.org>")
set(CPACK_SOURCE_IGNORE_FILES "${CPACK_SOURCE_IGNORE_FILES};${CMAKE_SOURCE_DIR}/.git;${CMAKE_BINARY_DIR}/*")
# if(DEFINED CMAKE_LIBRARY_OUTPUT_DIRECTORY)
#   set(CPACK_SOURCE_IGNORE_FILES "${CPACK_SOURCE_IGNORE_FILES};${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/*")
# endif()
if(DEFINED CMAKE_RUNTIME_OUTPUT_DIRECTORY)
  set(CPACK_SOURCE_IGNORE_FILES "${CPACK_SOURCE_IGNORE_FILES};${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/*")
endif()
# if(DEFINED CMAKE_ARCHIVE_OUTPUT_DIRECTORY)
#   set(CPACK_SOURCE_IGNORE_FILES "${CPACK_SOURCE_IGNORE_FILES};${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/*")
# endif()
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
link_libraries(${Boost_LIBRARIES} pthread)

add_executable(test_protocol tools/test_protocol.cpp)
add_executable(test_kopter tools/test_kopter.cpp)

install(DIRECTORY include/ 
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
# install(DIRECTORY lib/ 
#   DESTINATION lib)
install(DIRECTORY bin/ 
  DESTINATION bin
    FILE_PERMISSIONS
        OWNER_READ OWNER_EXECUTE OWNER_WRITE
        GROUP_READ GROUP_EXECUTE
        WORLD_READ WORLD_EXECUTE
    DIRECTORY_PERMISSIONS
        OWNER_READ OWNER_EXECUTE OWNER_WRITE
        GROUP_READ GROUP_EXECUTE
        WORLD_READ WORLD_EXECUTE
)
