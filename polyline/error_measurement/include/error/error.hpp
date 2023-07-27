#ifndef Error
#define Error

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include "../../utils/common.hpp"
#include "params/params.hpp"

//-----------------------------------------
//The difference in area between the original polyline and the simplified polyline.
double area(const std::vector<Point>& polyline) {
    double a = 0.0;
    for (int i = 0; i < polyline.size(); i++) {
        int j = (i + 1) % polyline.size();
        a += polyline[i].x * polyline[j].y - polyline[j].x * polyline[i].y;
    }
    return std::abs(a) / 2.0;
}

double areaDifference(const std::vector<Point>& polyline1,const std::vector<Point>& polyline2) {
    return area(polyline1) - area(polyline2);
}


//-------------------------

//The maximum distance between any point on the original polyline
// and its nearest point on the simplified polyline.



double hausdorffDistance(const std::vector<Point>& polyline1, const std::vector<Point>& polyline2) {
    double maxDistance = 0.0;
    for (Point p1 : polyline1) {
        double minDistance = std::numeric_limits<double>::infinity();
        for (Point p2 : polyline2) {
            double d = distance(p1, p2);
            if (d < minDistance) {
                minDistance = d;
            }
        }
        if (minDistance > maxDistance) {
            maxDistance = minDistance;
        }
    }
    return maxDistance;
}

//--------------------------------------
//The average difference in angle between consecutive segments of the original polyline and
// their corresponding segments in the simplified polyline.
double angle(Point p1, Point p2, Point p3) {
    double a = std::atan2(p2.y - p1.y, p2.x - p1.x);
    double b = std::atan2(p3.y - p2.y, p3.x - p2.x);
    double angle = std::abs(a - b);
    if (angle > M_PI) {
        angle = 2 * M_PI - angle;
    }
    return angle;
}

double angleError(std::vector<Point> polyline1, std::vector<Point> polyline2) {
    double totalAngleError = 0.0;
    int n = std::min(polyline1.size() - 2, polyline2.size() - 2);
    for (int i = 0; i < n; i++) {
        double a1 = angle(polyline1[i], polyline1[i + 1], polyline1[i + 2]);
        double a2 = angle(polyline2[i], polyline2[i + 1], polyline2[i + 2]);
        totalAngleError += std::abs(a1 - a2);
    }
    return totalAngleError / n;
}


//-------------------------

//The overall distance between any point on the original polyline
// and its nearest point on the simplified polyline.



double fullDistance(const std::vector<Point>& polyline1, const std::vector<Point>& polyline2) {
    double sum = 0.0;
    for (Point p1 : polyline1) {
        double minDistance = std::numeric_limits<double>::infinity();
        for (Point p2 : polyline2) {
            double d = distance(p1, p2);
            if (d < minDistance) {
                minDistance = d;
            }
        }
        sum += minDistance;
    }
    return sum;
}

//---------------------------------------------

void Compare(const std::vector<Point>& polyline1, const std::vector<Point>& polyline2, std::ofstream& file3){
    double areaDiff = areaDifference(polyline1, polyline2);
    double angleDiff = angleError(polyline1, polyline2);
    double hausdorffDiff = hausdorffDistance(polyline1, polyline2);
    double fullDiff = fullDistance(polyline1, polyline2);
    file3 << "Area difference: " << areaDiff << std::endl;
    file3 << "Angle difference: " << angleDiff << std::endl;
    file3 << "Hausdorff difference: " << hausdorffDiff << std::endl;
    file3 << "Full distance: " << fullDiff << std::endl;
}



void runErrorMeasurement(const ErrorParams& params) {
    std::vector<Point> polyline1, polyline2;
    std::ifstream file1(params.original), file2(params.compressed);
    std::ofstream file3(params.output);
    if (file1 && file2) {
        double x, y;
        while (file1 >> x >> y) {
            polyline1.emplace_back(x, y);
        }
        while (file2 >> x >> y) {
            polyline2.emplace_back(x, y);
        }
        Compare(polyline1, polyline2, file3);
    } else {
        std::cerr << "Failed to open input files." << std::endl;
        return;
    }
}

#endif //Error