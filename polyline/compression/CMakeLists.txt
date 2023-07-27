add_library(compression compression.cpp)
target_link_libraries(compression params)
set_property(TARGET compression PROPERTY CXX_STANDARD 20)
target_include_directories(compression PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )
install(TARGETS compression)