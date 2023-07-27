#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <error/error.hpp>
#include <compression/compression.hpp>
#include <tsp_path/tsp_path.hpp>
#include <visualizer/visualizer.hpp>
#include <winding_path/winding_path.hpp>
#include <params/params.hpp>
#include <doctest/doctest.h>


TEST_CASE("TEST_0") {
    WindingPathParams wind_params;
    wind_params.output = "original0.txt";
    wind_params.num_winds = 1;
    wind_params.seed = 3;
    wind_params.line_dot_cnt = 6;
    wind_params.circle_dot_cnt = 3;

    CompressionParams compress_params;
    compress_params.input = "original0.txt";
    compress_params.output = "compress0.txt";
    compress_params.tolerance = 1;
    compress_params.max_locs = 1;
    compress_params.seed = 2;

    ErrorParams error_params;
    error_params.original = "original0.txt";
    error_params.compressed = "compress0.txt";
    error_params.output = "error0.txt";

    generateWindingPath(wind_params);
    runCompression(compress_params);
    runErrorMeasurement(error_params);


    VisualizerParams visualizer_params;
    visualizer_params.original = "original0.txt";
    visualizer_params.compressed = "compress0.txt";
    visualizer_params.show_original = true;
    visualizer_params.show_compressed = true;

    visualize(visualizer_params);

}


TEST_CASE("TEST_1") {
    WindingPathParams wind_params;
    wind_params.output = "original1.txt";
    wind_params.num_winds = 2;
    wind_params.seed = 1;
    wind_params.epsilon = 200;
    wind_params.line_dot_cnt = 3;
    wind_params.circle_dot_cnt = 3;

    CompressionParams compress_params;
    compress_params.input = "original1.txt";
    compress_params.output = "compress1.txt";
    compress_params.tolerance = 0.2;
    compress_params.max_locs = 1;
    compress_params.seed = 2;

    ErrorParams error_params;
    error_params.original = "original1.txt";
    error_params.compressed = "compress1.txt";
    error_params.output = "error1.txt";

    generateWindingPath(wind_params);
    runCompression(compress_params);
    runErrorMeasurement(error_params);


    VisualizerParams visualizer_params;
    visualizer_params.original = "original1.txt";
    visualizer_params.compressed = "compress1.txt";
    visualizer_params.show_original = true;
    visualizer_params.show_compressed = true;

    visualize(visualizer_params);

}


TEST_CASE("TEST_2-convex") {
    TSP_PARAMS tsp_params;
    tsp_params.n = 100;
    tsp_params.seed = 1;
    tsp_params.from = 0;
    tsp_params.to = 100;
    tsp_params.output = "original2-convex.txt";

    generatePathTSP(tsp_params);

    VisualizerParams visualizer_params;
    visualizer_params.original = "original2-convex.txt";
    visualizer_params.compressed = "compress2-convex.txt";
    visualizer_params.show_original = true;
    visualizer_params.show_compressed = true;
    visualize(visualizer_params);

}


TEST_CASE("WIND-TEST_1") {
    WindingPathParams wind_params;
    wind_params.output = "wind-original1.txt";
    wind_params.num_winds = 3;
    wind_params.avoidance_factor = 4;
    wind_params.seed = 3;

    generateWindingPath(wind_params);

    VisualizerParams visualizer_params;
    visualizer_params.original = "wind-original1.txt";
    visualizer_params.compressed = "wind-compress1.txt";
    visualizer_params.show_original = true;
    visualizer_params.show_compressed = false;

    visualize(visualizer_params);

}


TEST_CASE("WIND-TEST_2") {
    WindingPathParams wind_params;
    wind_params.output = "wind-original2.txt";
    wind_params.num_winds = 4;
    wind_params.avoidance_factor = 5;
    wind_params.seed = 45;

    generateWindingPath(wind_params);

    VisualizerParams visualizer_params;
    visualizer_params.original = "wind-original2.txt";
    visualizer_params.compressed = "wind-compress2.txt";
    visualizer_params.show_original = true;
    visualizer_params.show_compressed = false;

    visualize(visualizer_params);

}


TEST_CASE("WIND-TEST_3") {
    WindingPathParams wind_params;
    wind_params.output = "wind-original3.txt";
    wind_params.num_winds = 5;
    wind_params.avoidance_factor = 4;
    wind_params.seed = 13;


    generateWindingPath(wind_params);

    VisualizerParams visualizer_params;
    visualizer_params.original = "wind-original3.txt";
    visualizer_params.compressed = "wind-compress3.txt";
    visualizer_params.show_original = true;
    visualizer_params.show_compressed = false;

    visualize(visualizer_params);

}


TEST_CASE("TSP-TEST") {
    TSP_PARAMS tsp_params;
    tsp_params.n = 500;
    tsp_params.seed = 1;
    tsp_params.from = 0;
    tsp_params.to = 100;
    tsp_params.output = "tsp-original.txt";

    generatePathTSP(tsp_params);

    VisualizerParams visualizer_params;
    visualizer_params.original = "tsp-original.txt";
    visualizer_params.compressed = "tsp-compress.txt";
    visualizer_params.show_original = true;
    visualizer_params.show_compressed = false;

    visualize(visualizer_params);

}
