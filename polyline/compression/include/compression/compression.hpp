#ifndef Compression
#define Compression

#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <random>
#include <iomanip>
#include <stack>
#include "../../utils/common.hpp"
#include <params/params.hpp>

static CompressionParams _Compression_params;
std::mt19937 gen;


Point INF_POINT(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());


struct Segment {
    Point start;
    Point end;

    Segment(Point _s, Point _e) : start(_s), end(_e) {}

    Segment(double _x1, double _y1, double _x2, double _y2) : start(_x1, _y1), end(_x2, _y2) {}
};


const double EPSILON = 1e-9;

bool areEqual(double x, double y) {
    return std::abs(x - y) < EPSILON;
}

double crossProduct(Point A, Point B, Point C) {
    double ABx = B.x - A.x;
    double ABy = B.y - A.y;
    double ACx = C.x - A.x;
    double ACy = C.y - A.y;
    return ABx * ACy - ABy * ACx;
}

struct PointHash {
    std::size_t operator()(const Point &p) const {
        return std::hash<double>()(p.x) ^ std::hash<double>()(p.y);
    }
};


bool operator==(const Point &p1, const Point &p2) {
    return p1.x == p2.x && p1.y == p2.y;
}


std::unordered_map<Point, std::vector<Point>, PointHash> grid;
std::vector<std::vector<Point>> convex_parts;


bool cmp(Point a, Point b) {
    return a.x < b.x || a.x == b.x && a.y < b.y;
}

bool cw(Point a, Point b, Point c) {
    return a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y) < 0;
}

bool ccw(Point a, Point b, Point c) {
    return a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y) > 0;
}


std::vector<std::vector<Point>> getAllSequentialCombinations(const std::vector<std::vector<Point>> &vecs) {
    std::vector<std::vector<Point>> result;

    if (vecs.empty()) {
        return result;
    }

    int n = vecs.size();
    std::vector<int> indices(n, 0);
    while (true) {
        std::vector<Point> temp;
        for (int i = 0; i < n; i++) {
            temp.push_back(vecs[i][indices[i]]);
        }
        result.push_back(temp);

        int k = n - 1;
        while (k >= 0 && indices[k] == vecs[k].size() - 1) {
            k--;
        }

        if (k < 0) {
            break;
        }

        indices[k]++;
        for (int i = k + 1; i < n; i++) {
            indices[i] = 0;
        }
    }

    return result;
}


void convex_hull(std::vector<Point> &a) {
    if (a.size() == 1) return;
    sort(a.begin(), a.end(), &cmp);
    Point p1 = a[0], p2 = a.back();
    std::vector<Point> up, down;
    up.push_back(p1);
    down.push_back(p1);
    for (size_t i = 1; i < a.size(); ++i) {
        if (i == a.size() - 1 || cw(p1, a[i], p2)) {
            while (up.size() >= 2 && !cw(up[up.size() - 2], up[up.size() - 1], a[i]))
                up.pop_back();
            up.push_back(a[i]);
        }
        if (i == a.size() - 1 || ccw(p1, a[i], p2)) {
            while (down.size() >= 2 && !ccw(down[down.size() - 2], down[down.size() - 1], a[i]))
                down.pop_back();
            down.push_back(a[i]);
        }
    }
    a.clear();
    for (auto i: up)
        a.push_back(i);
    for (size_t i = down.size() - 2; i > 0; --i)
        a.push_back(down[i]);
}


double triangleArea(double x1, double y1, double x2, double y2, double x3, double y3) {
    return std::abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
}

bool isPointInsideTriangle(double px, double py, double x1, double y1, double x2, double y2, double x3, double y3) {
    double A = triangleArea(x1, y1, x2, y2, x3, y3);

    double A1 = triangleArea(px, py, x1, y1, x2, y2);
    double A2 = triangleArea(px, py, x2, y2, x3, y3);
    double A3 = triangleArea(px, py, x3, y3, x1, y1);

    return areEqual((A1 + A2 + A3), A);
}

std::vector<Point> generateTriangularGrid(const Point &p, double tolerance, double q) {

    double x1 = p.x + tolerance * std::cos(M_PI / 2);
    double y1 = p.y + tolerance * std::sin(M_PI / 2);
    double x2 = p.x + tolerance * std::cos(M_PI / 2 + M_PI * 2 / 3);
    double y2 = p.y + tolerance * std::sin(M_PI / 2 + M_PI * 2 / 3);
    double x3 = p.x + tolerance * std::cos(M_PI / 2 + M_PI * 4 / 3);
    double y3 = p.y + tolerance * std::sin(M_PI / 2 + +M_PI * 4 / 3);

    std::vector<Point> vertices;
    double r = tolerance * q / 2;
    int numCircles = std::ceil(tolerance / (2 * r));
    double shift = numCircles % 2 == 0 ? r : 0;
    for (int i = -numCircles; i <= numCircles; i++) {
        for (int j = -numCircles; j <= numCircles; j++) {
            Point center = {p.x + i * 2 * r + shift, p.y + j * 2 * r + shift};

            if (isPointInsideTriangle(center.x, center.y, x1, y1, x2, y2, x3, y3)) {
                vertices.emplace_back(center.x, center.y);
            }
        }
    }

    return vertices;
}

bool isInsideConvexPolygon(Point p, std::vector<Point> polygon) {
    int n = polygon.size();
    for (int i = 0; i < n; i++) {
        if (crossProduct(polygon[i], polygon[(i + 1) % n], p) > 0) {
            return false;
        }
    }
    return true;
}


int isPointInConvexChain(Point p, const std::vector<std::vector<Point>> &chain) {
    for (int i = 0; i < chain.size(); i++) {
        if (isInsideConvexPolygon(p, chain[i])) {
            return i;
        }
    }
    return -1;
}

bool isSegmentInConvexChain(Segment s, const std::vector<std::vector<Point>> &chain) {
    std::stack<Segment> segmentStack;
    segmentStack.push(s);

    while (!segmentStack.empty()) {
        Segment currSegment = segmentStack.top();
        segmentStack.pop();

        int chainIndexStart = isPointInConvexChain(currSegment.start, chain);
        int chainIndexEnd = isPointInConvexChain(currSegment.end, chain);

        if (chainIndexStart == -1 || chainIndexEnd == -1) {
            return false;
        }

        if (chainIndexStart == chainIndexEnd) {
            return true;
        }


        if (distance(currSegment.start, currSegment.end) >= 1e-6) {
            Point midPoint = {(currSegment.start.x + currSegment.end.x) / 2,
                              (currSegment.start.y + currSegment.end.y) / 2};
            Segment s1 = {currSegment.start, midPoint};
            Segment s2 = {midPoint, currSegment.end};
            segmentStack.push(s1);
            segmentStack.push(s2);
        }
    }

    return false;
}


void makeGrid(const std::vector<Point> &points, double tolerance, double q) {
    for (auto &p: points) {
        grid.insert({p, generateTriangularGrid(p, tolerance, q)});
    }
}

void makeConvexHulls(const std::vector<Point> &points) {
    std::vector<Point> segments;
    for (int i = 0; i < points.size() - 1; ++i) {
        for (auto &gp: grid[points[i]])
            segments.emplace_back(gp.x, gp.y);
        for (auto &gp: grid[points[i + 1]])
            segments.emplace_back(gp.x, gp.y);
        convex_hull(segments);
        convex_parts.push_back(segments);
        segments.clear();
    }
}

struct Specimen {
    Point original;
    Point shifted;
};

double distance(Point p, Segment s) {
    double x = p.x;
    double y = p.y;
    double x1 = s.start.x;
    double y1 = s.start.y;
    double x2 = s.end.x;
    double y2 = s.end.y;

    double px = x2 - x1;
    double py = y2 - y1;
    double dAB = px * px + py * py;
    double u = ((x - x1) * px + (y - y1) * py) / dAB;

    x = x1 + u * px;
    y = y1 + u * py;


    return distance(p, {x, y});
}

double summed_distance_sq(const std::vector<Point> &points, Segment s) {
    double sum = 0;
    for (const Point& p: points) {
        sum += std::pow(distance(p, s), 2);
    }
    return sum;
}

double integralSquareDeviation(const std::vector<Point>& points,const std::vector<Specimen>& specimens) {
    int ptr = 0;
    double sum = 0;
    std::vector<Point> path;
    for (int i = 0; i < specimens.size() - 1; ++i) {
        auto start = specimens[i];
        auto end = specimens[i + 1];
        while (ptr < points.size() && !(points[ptr] == start.original)) {
            ++ptr;
        }
        while (ptr < points.size() && !(points[ptr] == end.original)) {
            path.push_back(points[ptr]);
            ++ptr;
        }
        if(ptr < points.size() && points[ptr] == end.original){
            path.push_back(points[ptr]);
        }
        sum += summed_distance_sq(path, {start.shifted, end.shifted});
        path.clear();
    }
    return sum;
}

template<typename T>
T getRandomFromVec(const std::vector<T> &vec) {
    std::uniform_int_distribution<> dis(0, vec.size() - 1);
    int randomIndex = dis(gen);
    return vec[randomIndex];
}

template<typename T>
std::vector<T> shuffleVector(std::vector<T> v) {
    std::shuffle(v.begin(), v.end(), gen);
    return v;
}


std::vector<std::vector<Specimen>> makePaths(const std::vector<Point> &points) {
    std::vector<std::vector<Point>> min_grid;
    for (int i = 0; i < points.size(); i++) {
        auto shuffle = shuffleVector(grid[points[i]]);
        min_grid.emplace_back(shuffle.begin(),
                              std::min(shuffle.begin() + (_Compression_params.max_locs - 1), shuffle.end()));
        min_grid.back().push_back(points[i]);
        if (i == points.size() - 1 or i == 0) continue;
        min_grid.back().push_back(INF_POINT);
    }

    auto paths = getAllSequentialCombinations(min_grid);
    std::cout << paths.size() << std::endl;
    std::vector<std::vector<Specimen>> result_paths;
    int ni = 0;
    for (auto &path: paths) {
        std::cout << ++ni << std::endl;
        std::vector<Specimen> new_path;
        std::vector<Specimen> del_part;
        for (int i = 0; i < path.size(); ++i) {
            if (!(path[i] == Point(INF_POINT))) {
                if (!del_part.empty()) {
                    if (!new_path.empty()) {
                        if (isSegmentInConvexChain(Segment(new_path.back().shifted, path[i]), convex_parts)) {
                        } else {
                            for (auto &item: del_part) {
                                new_path.push_back({item.original, item.original});
                            }
                        }
                    }
                    del_part.clear();
                }
                new_path.push_back({points[i], path[i]});
            } else {
                del_part.push_back({points[i], path[i]});
            }
        }
        result_paths.push_back(new_path);
    }

    return result_paths;
}


std::vector<Specimen> optimalPath(const std::vector<Point> &points, const std::vector<std::vector<Specimen>> &paths) {
    double minError = std::numeric_limits<double>::infinity();
    int minIndex = 0;
    int minVertexCnt = std::numeric_limits<int>::max();

    for (int i = 0; i < paths.size(); ++i) {
        std::cout << i << std::endl;
            double error = integralSquareDeviation(points, paths[i]);
            if ((std::abs(minError - error) >= EPSILON && minVertexCnt > paths[i].size())) {
                minError = error;
                minIndex = i;
                minVertexCnt = paths[i].size();
            }
    }
    return paths[minIndex];
}

std::vector<Point> Compress(const std::vector<Point> &points) {
    std::vector<Point> optimal;
    makeGrid(points, _Compression_params.tolerance, _Compression_params.q);
    makeConvexHulls(points);
    auto paths = makePaths(points);
    auto opt = optimalPath(points, paths);

    optimal.reserve(opt.size());
    for (auto &spec: opt) {
        optimal.push_back(spec.shifted);
    }
    return optimal;
}


void runCompression(const CompressionParams &params) {
    _Compression_params = params;
    gen = std::mt19937(_Compression_params.seed);

    std::vector<Point> polyline1;
    std::ifstream file1(params.input);
    if (file1) {
        double x, y;
        while (file1 >> x >> y) {
            polyline1.emplace_back(x, y);
        }
    } else {
        std::cerr << "Failed to open input files." << std::endl;
        return;
    }
    std::ofstream file2(params.output);
    file2.precision(10);
    if (!file2) {
        std::cerr << "Failed to open output file." << std::endl;
        return;
    }

    std::vector<Point> polyline2 = Compress(polyline1);
//    for (auto &vertices: convex_parts) {
//        for (int i = 0; i < vertices.size(); i++) {
//            file2 << vertices[i].x << " " << vertices[i].y << "\n";
//        }
//        file2 << vertices[0].x << " " << vertices[0].y << "\n";
//
//    }
    for (const auto &point: polyline2) {
        file2 << point.x << " " << point.y << "\n";
    }
}


#endif //Compression
