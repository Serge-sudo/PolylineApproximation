add_library(params params.cpp)
set_property(TARGET params PROPERTY CXX_STANDARD 20)
target_include_directories(params PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )
install(TARGETS params)