#include <cmath>

#include "AlchLib/util/point.hpp"

alchlib::Point2D::Point2D() {
    this->x = 0;
    this->y = 0;
    this->theta = 0;
}

alchlib::Point2D::Point2D(double x, double y) {
    this->x = x;
    this->y = y;
    this->theta = 0;
}

alchlib::Point2D::Point2D(double x, double y, double theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

alchlib::Point2D alchlib::Point2D::operator+(const Point2D& other) const {
    return Point2D(this->x + other.x, this->y + other.y);
}

alchlib::Point2D alchlib::Point2D::operator-(const Point2D& other) const {
    return Point2D(this->x - other.x, this->y - other.y);
}

alchlib::Point2D alchlib::Point2D::operator*(float scalar) const {
    return Point2D(this->x * scalar, this->y * scalar);
}

double alchlib::Point2D::operator*(const Point2D& other) const {
    return this->x * other.x + this->y * other.y;
}

double alchlib::Point2D::crossProduct(const Point2D& other) const {
    return this->x * other.y - this->y * other.x;
}

double alchlib::Point2D::distanceTo(const Point2D& other) const {
    float delta_x = this->x - other.x;
    float delta_y = this->y - other.y;
    return std::sqrt(delta_x * delta_x + delta_y * delta_y);
}