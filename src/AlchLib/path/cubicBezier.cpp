#include <cmath>

#include "AlchLib/util/math.hpp"
#include "AlchLib/path/cubicBezier.hpp"

alchlib::CubicBezier::CubicBezier(const Point2D& P1, const Point2D& P2, const Point2D& P3, const Point2D& P4) {
    this->P1 = P1;
    this->P2 = P2;
    this->P3 = P3;
    this->P4 = P4;
    computeDist();
}

void alchlib::CubicBezier::computeDist() {
    double sum = 0;
    const double D = 1000;
    for (int i = 0; i <= D; i++) {
        double t = i / D;
        Point2D d = this->getD(t);
        if (i != 0) {
            sum += std::sqrt(d.x * d.x + d.y * d.y) / D;
        }
        this->dist_to_t.t.push_back(t);
        this->dist_to_t.dist.push_back(sum);
    }
}

double alchlib::CubicBezier::getDistanceFromT(double t) const {
    return Math::interpolate(this->dist_to_t.t, this->dist_to_t.dist, t);
}

double alchlib::CubicBezier::getLength() {
    return this->dist_to_t.dist.back();
}

alchlib::Point2D alchlib::CubicBezier::getPoint(double t) const {
    const double u = 1 - t;

    Point2D point = this->P1 * (u * u * u);
    point = point + this->P2 * (3 * u * u * t);
    point = point + this->P3 * (3 * u * t * t);
    point = point + this->P4 * (t * t * t);

    return point;
}

alchlib::Pose alchlib::CubicBezier::getPose(double t) const {
    const double u = 1 - t;

    Point2D point = this->P1 * (u * u * u);
    point = point + this->P2 * (3 * u * u * t);
    point = point + this->P3 * (3 * u * t * t);
    point = point + this->P4 * (t * t * t);

    return Pose{ point.x, point.y, this->getHeading(t) };
}

alchlib::Point2D alchlib::CubicBezier::getD(double t) const {
    const double u = 1 - t;

    Point2D point = (this->P2 - this->P1) * (3 * u * u);
    point = point + (this->P3 - this->P2) * (6 * u * t);
    point = point + (this->P4 - this->P3) * (3 * t * t);

    return point;
}

double alchlib::CubicBezier::getHeading(double t) const {
    Point2D d = this->getD(t);
    return Math::normalize360(std::atan2(d.y, d.x), true);
}

alchlib::Point2D alchlib::CubicBezier::getDD(double t) const {
    const double u = 1 - t;

    Point2D point = (this->P3 + this->P1 + this->P2 * -2) * (6 * u);
    point = point + (this->P4 + this->P2 + this->P3 * -2) * (6 * t);

    return point;
}

double alchlib::CubicBezier::getCurvature(double t) const {
    Point2D d = this->getD(t);
    Point2D dd = this->getDD(t);

    double num = d.x * dd.y - d.y * dd.x;
    double den = d.x * d.x + d.y * d.y;
    den *= den * den;
    den = std::sqrt(den);

    return num / den;
}
