#pragma once

#include <vector>

#include "AlchLib/util/point.hpp"
#include "AlchLib/util/pose.hpp"

namespace alchlib {

class CubicBezier {
public:
    Point2D P1;
    Point2D P2;
    Point2D P3;
    Point2D P4;

    CubicBezier(const Point2D& P1, const Point2D& P2, const Point2D& P3, const Point2D& P4);

    double getLength();

    Point2D getPoint(double t) const;

    Pose getPose(double t) const;

    Point2D getD(double t) const;

    double getHeading(double t) const;

    Point2D getDD(double t) const;

    double getCurvature(double t) const;

    double getDistanceFromT(double t) const;
private:
    struct DistToT {
        std::vector<double> t;
        std::vector<double> dist;
    } dist_to_t;

    void computeDist();
};
}