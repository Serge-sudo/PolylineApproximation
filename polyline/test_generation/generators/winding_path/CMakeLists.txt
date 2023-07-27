add_library(winding_path winding_path.cpp)
target_link_libraries(winding_path params)
set_property(TARGET winding_path PROPERTY CXX_STANDARD 20)
target_include_directories(winding_path PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )
install(TARGETS winding_path)