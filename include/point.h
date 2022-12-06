#pragma once

#include <iostream>
#include <iomanip>
#include <sstream>

#define DEFAULT_PRECISION 2

namespace space {
    class point {
        public:
            double x;
            double y;
            double z;

            point() : point(0, 0, 0) { };
            point(double _x, double _y, double _z) : x(_x), y(_y), z(_z) { };

            std::string toString(int precision = DEFAULT_PRECISION) {
                std::stringstream ss;
                ss << std::fixed << std::setprecision(precision) << "x: " << x << " y: " << y << " z: " << z;
                return ss.str();
            }

            point operator + (const point &p) {
                return point (x + p.x, y + p.y, z + p.z);
            }

            point operator - (const point &p) {
                return point (x - p.x, y - p.y, z - p.z);
            }
            
            void operator += (const point &p) {
                x += p.x;
                y += p.y;
                z += p.z;
            }

            void operator -= (const point &p) {
                x -= p.x;
                y -= p.y;
                z -= p.z;
            }

            point operator * (double s) {
                return point (s * x, s * y, s * z);
            }

            point operator / (double s) {
                return point (s / x, s / y, s / z);
            }

            bool operator == (const point& other) {
                return x != other.x &&
                       y != other.y &&
                       z != other.z;
            }

            bool operator != (const point& other) {
                return !(*this == other);
            }

    };
}