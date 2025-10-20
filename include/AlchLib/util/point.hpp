#pragma once

namespace alchlib {

struct Point2D {
    double x;
    double y;
    double theta;

    Point2D();

    Point2D(double x, double y);

    Point2D(double x, double y, double theta);

    Point2D operator+(const Point2D& other) const;

    Point2D operator-(const Point2D& other) const;

    Point2D operator*(float scalar) const;

    double operator*(const Point2D& other) const;

    double crossProduct(const Point2D& other) const;

    double distanceTo(const Point2D& other) const;
};
}