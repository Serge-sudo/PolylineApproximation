#ifndef Visualizer
#define Visualizer
#include <unistd.h>
#include <string>
#include <params/params.hpp>
#include <iostream>


void visualize(const VisualizerParams &params) {

    char *python_args[] = {(char *) "python3", (char *) CMAKE_CURRENT_BINARY_DIR"/scripts/graph.py",
                           const_cast<char *>(params.original.c_str()),
                           const_cast<char *>(params.compressed.c_str()),
                           const_cast<char *>(std::to_string((int) params.show_original).c_str()),
                           const_cast<char *>(std::to_string((int) params.show_compressed).c_str()), nullptr};
    execvp("python3", python_args);
}

#endif //Visualizer
