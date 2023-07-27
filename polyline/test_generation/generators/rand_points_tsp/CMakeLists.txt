add_library(tsp_path tsp_path.cpp)
set_property(TARGET tsp_path PROPERTY CXX_STANDARD 20)
target_link_libraries(tsp_path params)
target_include_directories(tsp_path PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )
install(TARGETS tsp_path)