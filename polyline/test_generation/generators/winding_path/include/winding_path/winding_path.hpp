#ifndef WindingPath
#define WindingPath

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <fstream>
#include <algorithm>
#include <params/params.hpp>


static WindingPathParams _Winding_params;

static std::mt19937 rand_gen;

using std::array;
using std::vector;

std::tuple<double, double, double, double>
rotate_line(double x1, double y1, double x2, double y2, double rad, double tx, double ty) {
    x1 -= tx;
    y1 -= ty;
    x2 -= tx;
    y2 -= ty;

    double new_x1 = x1 * cos(rad) - y1 * sin(rad);
    double new_y1 = x1 * sin(rad) + y1 * cos(rad);
    double new_x2 = x2 * cos(rad) - y2 * sin(rad);
    double new_y2 = x2 * sin(rad) + y2 * cos(rad);

    new_x1 += tx;
    new_y1 += ty;
    new_x2 += tx;
    new_y2 += ty;

    return std::make_tuple(new_x1, new_y1, new_x2, new_y2);
}

bool intersect(const std::tuple<double, double, double> &c1, const std::tuple<double, double, double> &c2) {
    double x1, y1, r1, x2, y2, r2;
    std::tie(x1, y1, r1) = c1;
    std::tie(x2, y2, r2) = c2;
    double distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    return distance < r1 + r2;
}

std::vector<std::tuple<double, double, double>>
generate_circles(int num_winds, double min_radius, double max_radius) {

    std::vector<std::tuple<double, double, double>> circles;
    for (int i = 0; i < num_winds; i++) {
        while (true) {
            double radius = std::uniform_real_distribution<double>(min_radius, max_radius)(rand_gen);
            double x = std::uniform_real_distribution<double>(_Winding_params.min_x + radius,
                                                              _Winding_params.max_x - radius)(rand_gen);
            double y = std::uniform_real_distribution<double>(_Winding_params.min_y + radius,
                                                              _Winding_params.max_y - radius)(rand_gen);
            std::tuple<double, double, double> circle = std::make_tuple(x, y, radius);
            bool intersects = false;
            for (const auto &c: circles) {
                if (intersect(circle, c)) {
                    intersects = true;
                    break;
                }
            }
            if (!intersects) {
                circles.push_back(circle);
                break;
            }
        }
    }
    return circles;
}

std::tuple<double, double, double, double> get_chord(double x, double y, double r, double offset) {
    double new_x1 = x - sqrt(r * r - offset * offset);
    double new_y1 = y + offset;
    double new_x2 = x + sqrt(r * r - offset * offset);
    double new_y2 = y + offset;
    return std::make_tuple(new_x1, new_y1, new_x2, new_y2);
}


struct double_array_hash {
    inline std::size_t operator()(const std::array<double, 2> &arr) const {
        std::size_t seed = 0;
        for (const double &val: arr) {
            std::hash<double> hasher;
            seed ^= hasher(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};


std::vector<std::array<double, 2>> Points_ends;
std::unordered_map<std::array<double, 2>, std::array<std::array<std::array<double, 2>, 2>, 2>, double_array_hash> Parallel_lines;


void draw_lines(const std::vector<std::tuple<double, double, double>> &circles) {

    for (const auto &circle: circles) {
        double x = std::get<0>(circle);
        double y = std::get<1>(circle);
        double r = std::get<2>(circle);

        double angle = std::uniform_real_distribution<double>(0, M_PI)(rand_gen);
        double offset = -r / 3;
        std::array<std::array<std::array<double, 2>, 2>, 2> pair;

        auto [x1, y1, x2, y2] = rotate_line(std::get<0>(get_chord(x, y, r, offset)),
                                            std::get<1>(get_chord(x, y, r, offset)),
                                            std::get<2>(get_chord(x, y, r, offset)),
                                            std::get<3>(get_chord(x, y, r, offset)), angle, x, y);
        pair[0][0] = {{x1, y1}};
        pair[0][1] = {{x2, y2}};

        offset = r / 3;
        std::tie(x1, y1, x2, y2) = rotate_line(std::get<0>(get_chord(x, y, r, offset)),
                                               std::get<1>(get_chord(x, y, r, offset)),
                                               std::get<2>(get_chord(x, y, r, offset)),
                                               std::get<3>(get_chord(x, y, r, offset)), angle, x, y);
        pair[1][0] = {{x1, y1}};
        pair[1][1] = {{x2, y2}};

        Parallel_lines[{pair[0][0]}] = pair;
        Parallel_lines[{pair[1][1]}] = pair;
        Points_ends.push_back(pair[0][0]);
        Points_ends.push_back(pair[1][1]);
    }
}

std::vector<std::array<double, 2>>
generate_random_points(std::array<double, 2> a, std::array<double, 2> b, double epsilon, int n_points) {
    std::vector<std::array<double, 2>> random_points;


    for (int i = 0; i < n_points; i++) {
        double random_distance = std::uniform_real_distribution<double>(0, epsilon)(rand_gen);

        std::array<double, 2> random_offset = {{std::uniform_real_distribution<double>(-1, 1)(rand_gen),
                                                std::uniform_real_distribution<double>(-1, 1)(rand_gen)}};
        double norm = std::sqrt(random_offset[0] * random_offset[0] + random_offset[1] * random_offset[1]);
        std::array<double, 2> unit_direction = {{random_offset[0] / norm, random_offset[1] / norm}};

        std::array<double, 2> random_point = {
                {a[0] + random_distance * unit_direction[0], a[1] + random_distance * unit_direction[1]}};
        random_points.push_back(random_point);
    }

    return random_points;
}


double heuristic(std::array<double, 2> point, std::array<double, 2> b,
                 const std::unordered_set<std::array<double, 2>, double_array_hash> &E,
                 double avoidance_factor) {
    double distance_to_b = std::sqrt(pow(point[0] - b[0], 2) + pow(point[1] - b[1], 2));
    double boundary = 0;
    if (distance_to_b > _Winding_params.max_distance)
        boundary = distance_to_b - _Winding_params.max_distance;
    double avoidance_sum = 0;
    for (const auto &e: E) {
        if (!(e[0] == b[0] && e[1] == b[1])) {
            double val = std::sqrt(pow(point[0] - e[0], 2) + pow(point[1] - e[1], 2));
            avoidance_sum += val < 1e9 ? 1 / val : 1e9;
        }
    }
    return distance_to_b + avoidance_factor * avoidance_sum + _Winding_params.boundary_factor * boundary;
}

std::array<double, 2>
find_optimal_point(const std::vector<std::array<double, 2>> &random_points, std::array<double, 2> b,
                   const std::unordered_set<std::array<double, 2>, double_array_hash> &E, double avoidance_factor) {
    return *std::min_element(random_points.begin(), random_points.end(), [&](const auto &p1, const auto &p2) {
        return heuristic(p1, b, E, avoidance_factor) < heuristic(p2, b, E, avoidance_factor);
    });
}

std::vector<std::array<double, 2>>
connect_points(std::array<double, 2> a, std::array<double, 2> b,
               const std::unordered_set<std::array<double, 2>, double_array_hash> &E,
               double epsilon, int n_points, double avoidance_factor) {
    std::vector<std::array<double, 2>> curve_points = {a};
    std::array<double, 2> current_point = a;

    while (std::sqrt(pow(current_point[0] - b[0], 2) + pow(current_point[1] - b[1], 2)) > epsilon) {
        std::vector<std::array<double, 2>> random_points = generate_random_points(current_point, b, epsilon, n_points);
        std::array<double, 2> optimal_point = find_optimal_point(random_points, b, E, avoidance_factor);
        curve_points.push_back(optimal_point);
        current_point = optimal_point;
    }
    curve_points.push_back(b);
    return curve_points;
}


array<double, 2> point_on_line(const array<double, 2> &p1, const array<double, 2> &p2, double d) {
    array<double, 2> v = {p2[0] - p1[0], p2[1] - p1[1]};
    double norm = std::sqrt(v[0] * v[0] + v[1] * v[1]);
    v[0] /= norm;
    v[1] /= norm;

    array<double, 2> pos = {p2[0] - d * v[0], p2[1] - d * v[1]};
    return pos;
}

double get_angle(const array<double, 2> &p1, const array<double, 2> &p2) {
    array<double, 2> vec = {p2[0] - p1[0], p2[1] - p1[1]};
    if (vec[0] < 0 && vec[1] < 0) {
        return -(M_PI - std::atan(vec[1] / vec[0]));
    } else if (vec[0] < 0 && vec[1] > 0) {
        return (M_PI + std::atan(vec[1] / vec[0]));
    }
    return std::atan(vec[1] / vec[0]);
}

double point_to_line_distance(const array<double, 2> &point, const array<double, 2> &line_point_1,
                              const array<double, 2> &line_point_2) {
    return std::abs((line_point_2[1] - line_point_1[1]) * point[0] - (line_point_2[0] - line_point_1[0]) * point[1] +
                    line_point_2[0] * line_point_1[1] - line_point_2[1] * line_point_1[0]) / std::sqrt(
            (line_point_2[1] - line_point_1[1]) * (line_point_2[1] - line_point_1[1]) +
            (line_point_2[0] - line_point_1[0]) * (line_point_2[0] - line_point_1[0]));
}

vector<array<double, 2>>
winding_path(array<double, 2> p1, array<double, 2> p2, array<double, 2> p3, array<double, 2> p4) {
    array<double, 2> c1, c2, c3, c4;
    double angle;

    auto draw_circle = [&](const array<double, 2> &center, double radius, bool flag) {
        vector<array<double, 2>> circle_points;

        double start_theta, end_theta;
        if (angle > 0) {
            start_theta = -(M_PI / 2 - angle);
            end_theta = M_PI - (M_PI / 2 - angle);
            if (flag) {
                start_theta += M_PI;
                end_theta += M_PI;
            }
        } else {
            start_theta = -(M_PI / 2 - angle);
            end_theta = M_PI - (M_PI / 2 - angle);
            if (!flag) {
                start_theta += M_PI;
                end_theta += M_PI;
            }
        }

        double theta_step = (end_theta - start_theta) / (_Winding_params.circle_dot_cnt - 1);
        for (int i = 0; i < _Winding_params.circle_dot_cnt; ++i) {
            double t = start_theta + i * theta_step;
            circle_points.push_back({center[0] + radius * std::cos(t), center[1] + radius * std::sin(t)});
        }

        if (flag) {
            c1 = circle_points.front();
            c2 = circle_points.back();
        } else {
            c3 = circle_points.front();
            c4 = circle_points.back();
        }

        return circle_points;
    };

    array<double, 2> p5 = {(p1[0] + p3[0]) / 2, (p1[1] + p3[1]) / 2};
    array<double, 2> p6 = {(p2[0] + p4[0]) / 2, (p2[1] + p4[1]) / 2};

    array<double, 2> p7 = {(p1[0] + p5[0]) / 2, (p1[1] + p5[1]) / 2};
    array<double, 2> p8 = {(p2[0] + p6[0]) / 2, (p2[1] + p6[1]) / 2};

    array<double, 2> p9 = {(p5[0] + p3[0]) / 2, (p5[1] + p3[1]) / 2};
    array<double, 2> p10 = {(p6[0] + p4[0]) / 2, (p6[1] + p4[1]) / 2};

    double r = point_to_line_distance(p1, p5, p6) / 2;

    angle = get_angle(p1, p2);

    vector<array<double, 2>> circle1 = draw_circle(point_on_line(p7, p8, r), r, angle <= 0);
    vector<array<double, 2>> circle2 = draw_circle(point_on_line(p10, p9, r), r, angle > 0);

    vector<array<double, 2>> new_res;

    if (angle > 0) {
        for (int i = 0; i < _Winding_params.line_dot_cnt; ++i) {
            double t = static_cast<double>(i) / (_Winding_params.line_dot_cnt - 1);
            new_res.push_back({p1[0] + t * (c4[0] - p1[0]), p1[1] + t * (c4[1] - p1[1])});
        }
        new_res.insert(new_res.end(), circle1.rbegin(), circle1.rend());
        for (int i = 0; i < _Winding_params.line_dot_cnt; ++i) {
            double t = static_cast<double>(i) / (_Winding_params.line_dot_cnt - 1);
            new_res.push_back({c1[0] + t * (c3[0] - c1[0]), c1[1] + t * (c3[1] - c1[1])});
        }
        new_res.insert(new_res.end(), circle2.begin(), circle2.end());
        for (int i = 0; i < _Winding_params.line_dot_cnt; ++i) {
            double t = static_cast<double>(i) / (_Winding_params.line_dot_cnt - 1);
            new_res.push_back({p4[0] + t * (c2[0] - p4[0]), p4[1] + t * (c2[1] - p4[1])});
        }
    } else {
        for (int i = 0; i < _Winding_params.line_dot_cnt; ++i) {
            double t = static_cast<double>(i) / (_Winding_params.line_dot_cnt - 1);
            new_res.push_back({p4[0] + t * (c3[0] - p4[0]), p4[1] + t * (c3[1] - p4[1])});
        }
        new_res.insert(new_res.end(), circle2.begin(), circle2.end());
        for (int i = 0; i < _Winding_params.line_dot_cnt; ++i) {
            double t = static_cast<double>(i) / (_Winding_params.line_dot_cnt - 1);
            new_res.push_back({c4[0] + t * (c2[0] - c4[0]), c4[1] + t * (c2[1] - c4[1])});
        }
        new_res.insert(new_res.end(), circle1.rbegin(), circle1.rend());
        for (int i = 0; i < _Winding_params.line_dot_cnt; ++i) {
            double t = static_cast<double>(i) / (_Winding_params.line_dot_cnt - 1);
            new_res.push_back({c1[0] + t * (p1[0] - c1[0]), c1[1] + t * (p1[1] - c1[1])});
        }
    }

    return new_res;
}


std::vector<std::pair<int, int>> generate_connection_list(int size) {
    std::vector<std::array<int, 2>> arr(size / 2);

    int cnt = 0;
    for (int i = 0; i < size / 2; ++i) {
        arr[i] = {cnt++, cnt++};
    }
    std::shuffle(arr.begin(), arr.end(), rand_gen);
    for (auto &item: arr) {
        std::shuffle(item.begin(), item.end(), rand_gen);
    }
    std::vector<std::pair<int, int>> res;
    bool cur = false;
    for (int i = 0; i < size / 2 - 1; ++i) {
        res.emplace_back(arr[i][int(cur)], arr[i + 1][int(cur)]);
        cur = !cur;
    }
    res.emplace_back(arr[size / 2 - 1][int(cur)], -1);
    return res;
}


void generateWindingPath(WindingPathParams &p) {
    rand_gen = std::mt19937(p.seed);
    _Winding_params = p;
    auto circles = generate_circles(_Winding_params.num_winds, _Winding_params.min_radius, _Winding_params.max_radius);
    draw_lines(circles);
    std::unordered_set<std::array<double, 2>, double_array_hash> points_plane;
    std::vector<std::array<double, 2>> result;
    std::vector<std::pair<int, int>> connection_list = generate_connection_list(2*_Winding_params.num_winds);

    for (const auto &pair: connection_list) {

        const std::array<double, 2> &point1 = Points_ends.at(pair.first);
        const auto &lines = Parallel_lines.at(point1);
        std::array<double, 2> p1 = lines[0][0];
        std::array<double, 2> p2 = lines[0][1];
        std::array<double, 2> p3 = lines[1][0];
        std::array<double, 2> p4 = lines[1][1];
        std::vector<std::array<double, 2>> path = winding_path(p4, p3, p2, p1);
        points_plane.insert(path.begin(), path.end());

    }
    for (const auto &pair: connection_list) {
        const std::array<double, 2> &point1 = Points_ends.at(pair.first);
        const auto &lines = Parallel_lines.at(point1);
        std::array<double, 2> p1 = lines[0][0];
        std::array<double, 2> p2 = lines[0][1];
        std::array<double, 2> p3 = lines[1][0];
        std::array<double, 2> p4 = lines[1][1];
        std::vector<std::array<double, 2>> path = winding_path(p4, p3, p2, p1);
        if (path.back() != point1) {
            std::reverse(path.begin(), path.end());
        }
        for (auto it = path.begin(); it != path.end(); ++it) {
            result.push_back(*it);
        }
        if (pair.second == -1) {
            break;
        }
        const std::array<double, 2> &point2 = Points_ends.at(pair.second);
        std::vector<std::array<double, 2>> curve_points = connect_points(point1, point2, points_plane,
                                                                         _Winding_params.epsilon,
                                                                         _Winding_params.rand_points_on_each_step,
                                                                         _Winding_params.avoidance_factor);
        points_plane.insert(curve_points.begin(), curve_points.end());
        result.insert(result.end(), curve_points.begin(), curve_points.end());
    }
    std::ofstream output_file(_Winding_params.output);
    auto newEnd = std::unique(result.begin(), result.end());
    result.erase(newEnd, result.end());
    for (const auto &point: result) {
        output_file << point[0] << " " << point[1] << std::endl;
    }
    std::cout << "Done" << std::endl;
}

#endif //WindingPath