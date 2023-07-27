#ifndef PARAMS
#define PARAMS
#include <string>
struct TSP_PARAMS{
    int n = 300;
    int seed= 42;
    int from = 5;
    int to =   15;
    std::string output = "original.txt";
};

struct WindingPathParams{
    int max_x = 500;
    int max_y = 500;
    int min_x = 0;
    int min_y = 0;
    int seed= 15;
    double epsilon = 5;
    int rand_points_on_each_step = 25;
    double avoidance_factor = 3;
    int line_dot_cnt = 20;
    int num_winds = 5;
    double min_radius = 20;
    double max_radius = 50;
    int circle_dot_cnt = 10;
    double max_distance = 500;
    double boundary_factor = 3;
    std::string output = "original.txt";
};

struct ErrorParams{
    std::string original = "original.txt";
    std::string compressed = "compressed.txt";
    std::string output = "error.txt";
};

struct CompressionParams {
    double tolerance = 0.2;
    double q = 0.3;
    int seed = 0;
    int max_locs = 10;
    std::string input = "original.txt";
    std::string output = "compressed.txt";
};

struct VisualizerParams {
    std::string original = "original.txt";
    std::string compressed = "compressed.txt";
    bool show_original = true;
    bool show_compressed = true;
};

#endif //PARAMS