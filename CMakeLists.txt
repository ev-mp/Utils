cmake_minimum_required(VERSION 2.8)

project(Utils LANGUAGES CXX C)
add_subdirectory(Basic_depth)
add_subdirectory(Metadata_node_streaming)

# Update system cache with newly installed binaries/libraries
if(UNIX)
    install(CODE "execute_process(COMMAND ldconfig)")
endif()
