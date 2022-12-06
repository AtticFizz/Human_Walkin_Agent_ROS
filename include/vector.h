#pragma once

#include <iostream>
#include <math.h>
#include <cmath>

#include "point.h"
#include "utils.cpp"

namespace space {
    class vector : public point {
        public:
            vector() : vector(0, 0, 0) { };
            vector(double _x, double _y, double _z) : point(_x, _y, _z) { };

            double length() {
                return sqrt((x * x) + (y * y) + (z * z));
            }

            vector normalized() {
                double len = length();
                return vector (x / len, y / len, z / len);
            }

            double angle(vector v) {
                return y * v.y - x * v.x;
            }

            void rotateAroundX(double radians) {
                *this = rotatedAroundX(radians);
            }
            vector rotatedAroundX(double radians) {
                double y_rx = (cos(radians) * y) - (sin(radians) * z);
                double z_rx = (cos(radians) * z) + (sin(radians) * y);

                return vector(x, y_rx, z_rx);
            }

            void rotateAroundY(double radians) {
                *this = rotatedAroundY(radians);
            }
            vector rotatedAroundY(double radians) {
                double x_ry = (cos(radians) * x) + (sin(radians) * z);
                double z_ry = (cos(radians) * z) - (sin(radians) * x);

                return vector(x_ry, y, z_ry);
            }

            void rotateAroundZ(double radians) {
                *this = rotatedAroundZ(radians);
            }
            vector rotatedAroundZ(double radians) {
                double x_rz = (cos(radians) * x) - (sin(radians) * y);
                double y_rz = (cos(radians) * y) + (sin(radians) * x);
  
                return vector(x_rz, y_rz, z);
            }

            std::string toString(int precision = DEFAULT_PRECISION) {
                std::stringstream ss;
                ss << std::fixed << std::setprecision(precision) << "x: " << x << " y: " << y << " z: " << z << " length: " << length();
                return ss.str();
            }

            vector operator + (const vector &v) {
                return vector (x + v.x, y + v.y, z + v.z);
            }

            vector operator - (const vector &v) {
                return vector (x - v.x, y - v.y, z - v.z);
            }

            vector operator * (double s) {
                return vector (s * x, s * y, s * z);
            }

            vector operator / (double s) {
                return vector (s / x, s / y, s / z);
            }

            bool operator == (const vector& other) {
                return x != other.x &&
                       y != other.y &&
                       z != other.z;
            }

            bool operator != (const vector& other) {
                return !(*this == other);
            }

    };
}