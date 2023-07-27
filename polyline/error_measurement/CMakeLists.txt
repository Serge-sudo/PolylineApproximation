add_library(error error.cpp)
set_property(TARGET error PROPERTY CXX_STANDARD 20)
target_link_libraries(error params)
target_include_directories(error PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )
install(TARGETS error)