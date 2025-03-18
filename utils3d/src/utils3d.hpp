#pragma once

#include <concepts>
#include <limits>

#include <cmath>
#include <algorithm>
#include <expected>

/** @brief Checks if two floating-point numbers are equal with the given epsilon (machine epsilon by default)
*/
template <std::floating_point T>
constexpr inline bool equal(const T a, const T b, const T epsilon = std::numeric_limits<T>::epsilon()) {
    return std::fabs(a - b) < epsilon;
}

/** @brief 3D Utils
*/
namespace utils3D {

    /** @brief Vector representation in 3D
    */
    class Vector {
    public:
        double x, y, z;

        Vector(const double x = 0.0, const double y = 0.0, const double z = 0.0)
            : x(x), y(y), z(z) {}

        /** @brief Dot product of two vectors
        */
        double dot(const Vector& other) const {
            return x * other.x + y * other.y + z * other.z;
        }

        /** @brief Cross product of two vectors
        */
        Vector cross(const Vector& other) const {
            return Vector(
                y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x
            );
        }

        /** @brief Length of the vector
        */
        double length() const {
            return std::sqrt(x * x + y * y + z * z);
        }
    }; // class Vector

    /** @brief Point representation in 3D
    */
    class Point {
    public:
        double x, y, z;

        Point(const double x = 0.0, const double y = 0.0, const double z = 0.0)
            : x(x), y(y), z(z) {}

        /** @brief Subtraction of two points
         *  @return A vector from one point to another
        */
        Vector operator-(const Point& other) const {
            return Vector(x - other.x, y - other.y, z - other.z);
        }

        /** @brief Comparison of two points
        */
        constexpr bool operator==(const Point& other) const {
            return equal(this->x, other.x) && equal(this->y, other.y) && equal(this->z, other.z);
        }
    }; // class Point
    
    /** @brief Distance between two points
    */
    double distance(const Point& p1, const Point& p2) {
        return (p2 - p1).length();
    }

    /** @brief Segment representation in 3D
    */
    class Segment {
    public:
        Point p1, p2;

        Segment(const Point& p1, const Point& p2)
            : p1(p1), p2(p2) {}

        /** @brief Direction vector of the segment
        */
        Vector direction() const {
            return p2 - p1;
        }

        /** @brief Checks if the segment contains the point.
         * See "triangle inequality" for details (when c = a + b, - all three vertices lie on the same line)
        */
        bool contains(const Point& p) const {
            const double c = (p2 - p1).length();
            const double a = (p  - p1).length();
            const double b = (p2 -  p).length();

            return equal(c, a + b);
        }
    }; // class Segment

    /** @brief Minimal distance between endpoints of the two segments
    */
    double min_dist_between_endpoints(const Segment& s1, const Segment& s2) {
        return std::min({distance(s1.p1, s2.p1), distance(s1.p1, s2.p2),
                         distance(s1.p2, s2.p1), distance(s1.p2, s2.p2)});
    }

    /** @brief Distance between two segments
    */
    std::expected<double, std::string> distance(const Segment& s1, const Segment& s2) {

        /*  We need to make parametric equations for the first and second segments:

            * A point P(t) on the first segment can be represented as:
            ** P(t) = s1.p1 + u*t, where:
            *** u - direction vector of s1 (s1.p2 - s1.p1)
            *** t - parameter, ranging [0, 1]

            * A point Q(t) on the second segment can be represented as:
            ** Q(t) = s2.p1 + v*s, where:
            *** v - direction vector of s2 (s2.p2 - s2.p1)
            *** s - parameter, ranging [0, 1]

            * Squared Euclidean distance between these two points:
            ** D(t,s)^2 = (P(t) - Q(t))^2 = ((s1.p1 + u*t) - (s2.p1 + v*s))^2 = ((s1.p1 - s2.p1) + u*t - v*s))^2,
            * or, if define a vector w = (s1.p1 - s2.p1):
            ** D(t,s)^2 = (w + u*t - v*s)^2,

            * Then, we get a linear system of next equations, minimising D(t,s)^2 function
            * (partial derivatives with t and s should be equal to 0):
            ** d(D(t,s)^2)/dt = 0;
            ** d(D(t,s)^2)/ds = 0,
            * from which optimal t and s params are deduced analytically (see formulas in the code below) */

        // Vectors needed for the equations
        const Vector u = s1.direction();   // direction vector for the first segment
        const Vector v = s2.direction();   // direction vector for the second segment
        const Vector w = s1.p1 - s2.p1;    // vector between the two segment endpoints

        // Check neither of the segments is a point
        // TODO support such case
        if (equal(u.length(), 0.0) || equal(v.length(), 0.0)) {
            return std::unexpected("Some of the segments is a point");
        }

        // Constants needed for the equations
        const double a = u.dot(u);
        const double b = u.dot(v);
        const double c = v.dot(v);
        const double d = u.dot(w);
        const double e = v.dot(w);

        // Determinant from the Cramer's rule
        const double det = a * c - b * b;
        if (equal(det, 0.0)) {
            // The linear system has no unique solution for (t,s) parameters - meaning the segments're
            // collinear (parallel / overlapping / on the same line without overlapping)

            // Check for the partly / or fully overlapping case (if some of the
            // endpoints of one segment lie inside the another segment) - distance is 0 in such case
            if (s1.contains(s2.p1) || s1.contains(s2.p2) ||
                s2.contains(s1.p1) || s2.contains(s1.p2)) {
                return 0.0;
            }

            // On the same line (without overlapping) / or parallel case -
            // - find the minimum of distances between segments' endpoints in such case
            return min_dist_between_endpoints(s1, s2);
        }

        // Parameter of the optimal point on each segment
        double t = (b * e - c * d) / det;
        double s = (a * e - b * d) / det;

        // Clamp the values to the segment range [0, 1]
        t = std::clamp(t, 0.0, 1.0);
        s = std::clamp(s, 0.0, 1.0);

        // Find the optimal points on both segments
        const Point opt1 = Point(s1.p1.x + t * u.x, s1.p1.y + t * u.y, s1.p1.z + t * u.z);
        const Point opt2 = Point(s2.p1.x + s * v.x, s2.p1.y + s * v.y, s2.p1.z + s * v.z);
        const double dist_opt = distance(opt1, opt2);

        // As we're clamping the values "blindly" (while the optimal solution is found
        // for the case of unlimited lines, - not our limited segments),
        // We need to additionally check the distances between the segments' endpoints, - because there're some cases
        // when they can be less then the optimal value we got (as example, when clamping the "wrong" end -
        // the next test case: seg1((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)); seg2((2.0, 1.0, 0.0), (4.0, 1.5, 0.0)) ->
        // -> optimal t = s = -2, and both t and s will be clamped to 0,
        // while the min destance endpoint of the seg1 is the opposite one, - with t = 1)
        const double dist_min = std::min(dist_opt, min_dist_between_endpoints(s1, s2));
        return dist_min;
    }
} // namespace utils3D
