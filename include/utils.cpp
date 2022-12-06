#pragma once

#include <iostream>
#include <cmath>

namespace math {
    
    inline double deg2rad(double degrees) {
        return degrees * 4.0 * atan(1.0) / 180.0;
    }

    inline double rad2deg(double radians) {
        return radians * (180 / M_PI);
    }

    inline double map(double x1, double x2, double y1, double y2, double xi) {
        return (y2 - y1) / (double)(x2 - x1) * (xi * x1) + y1;
    }
}