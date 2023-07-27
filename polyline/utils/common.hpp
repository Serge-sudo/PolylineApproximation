#ifndef Common
#define Common


struct Point {
    double x, y;
    Point(double _x, double _y) : x(_x), y(_y) {}
};

double distance(Point p1, Point p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}



#endif //Common