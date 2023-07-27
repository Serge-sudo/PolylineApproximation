#ifndef TspPath
#define TspPath
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <numeric>
#include <random>
#include <algorithm>
#include <params/params.hpp>

using namespace std;

double euclidean_distance(const vector<double> &p1, const vector<double> &p2) {
    double sum = 0;
    for (size_t i = 0; i < p1.size(); ++i) {
        sum += pow(p1[i] - p2[i], 2);
    }
    return sqrt(sum);
}

vector<vector<double>> create_distance_matrix(const vector<vector<double>> &points) {
    size_t n = points.size();
    vector<vector<double>> distance_matrix(n, vector<double>(n));

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            distance_matrix[i][j] = euclidean_distance(points[i], points[j]);
        }
    }

    return distance_matrix;
}

double path_distance(const vector<int> &route, const vector<vector<double>> &distance_matrix) {
    double sum = 0;
    for (size_t p = 0; p < route.size() - 1; ++p) {
        sum += distance_matrix[route[p + 1]][route[p]];
    }
    return sum;
}

vector<int> two_opt_swap(const vector<int> &route, int i, int k) {
    vector<int> new_route(route.size());
    copy(route.begin(), route.begin() + i, new_route.begin());
    std::reverse_copy(route.begin() + i, route.begin() + k + 1, new_route.begin() + i);
    copy(route.begin() + k + 1, route.end(), new_route.begin() + k + 1);

    return new_route;
}

vector<int> two_opt(const vector<vector<double>> &distance_matrix, const vector<vector<double>> &cities,
                    double improvement_threshold) {
    vector<int> route(cities.size());
    iota(route.begin(), route.end(), 0);

    double improvement_factor = 1;
    double best_distance = path_distance(route, distance_matrix);
    while (improvement_factor > improvement_threshold) {
        double distance_to_beat = best_distance;
        for (int swap_first = 1; swap_first < static_cast<int>(route.size()) - 3; ++swap_first) {
            for (int swap_last = swap_first + 1; swap_last < static_cast<int>(route.size()) - 1; ++swap_last) {
                vector<int> new_route = two_opt_swap(route, swap_first, swap_last);
                double new_distance = path_distance(new_route, distance_matrix);
                if (new_distance < best_distance) {
                    route = new_route;
                    best_distance = new_distance;
                }
            }
        }
        improvement_factor = 1 - best_distance / distance_to_beat;
    }
    return route;
}


void generatePathTSP(const TSP_PARAMS& params){


    std::mt19937 rand_gen(params.seed);
    uniform_real_distribution<double> distribution(params.from, params.to);

    vector<vector<double>> cities(params.n, vector<double>(2));
    for (vector<double> &city: cities) {
        city[0] = distribution(rand_gen);
        city[1] = distribution(rand_gen);
    }

    vector<vector<double>> distance_matrix = create_distance_matrix(cities);

    vector<int> route = two_opt(distance_matrix, cities, 0.001);

    ofstream file(params.output);
    if (file.is_open()) {
        for (int index: route) {
            file << cities[index][0] << " " << cities[index][1] << "\n";
        }
        file.close();
    }
    cout << "Done" << endl;
}

#endif //TspPath