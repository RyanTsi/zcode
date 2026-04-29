#ifndef ZCODE_BVH3D_BVH3D_IMPL_H
#define ZCODE_BVH3D_BVH3D_IMPL_H

#ifndef ZCODE_BVH3D_BVH3D_H
#error "Include bvh3d.h instead of bvh3d_impl.h directly."
#endif

namespace zcode {
namespace bvh3d {

template <typename T>
constexpr Vec3<T>::Vec3() = default;

template <typename T>
constexpr Vec3<T>::Vec3(T x_value, T y_value, T z_value) : x(x_value), y(y_value), z(z_value) {}

template <typename T>
constexpr Vec3<T> operator+(const Vec3<T>& left, const Vec3<T>& right) {
    return Vec3<T>(left.x + right.x, left.y + right.y, left.z + right.z);
}

template <typename T>
constexpr Vec3<T> operator-(const Vec3<T>& left, const Vec3<T>& right) {
    return Vec3<T>(left.x - right.x, left.y - right.y, left.z - right.z);
}

template <typename T>
constexpr Vec3<T> operator*(const Vec3<T>& value, T scale) {
    return Vec3<T>(value.x * scale, value.y * scale, value.z * scale);
}

template <typename T>
constexpr Vec3<T> operator/(const Vec3<T>& value, T scale) {
    return Vec3<T>(value.x / scale, value.y / scale, value.z / scale);
}

template <typename T>
constexpr Aabb3<T> Aabb3<T>::empty() {
    return Aabb3<T>{
        Vec3<T>(std::numeric_limits<T>::max(), std::numeric_limits<T>::max(), std::numeric_limits<T>::max()),
        Vec3<T>(std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest())};
}

template <typename T>
constexpr bool Aabb3<T>::is_valid() const {
    return lower.x <= upper.x && lower.y <= upper.y && lower.z <= upper.z;
}

template <typename T>
void Aabb3<T>::expand(const Vec3<T>& point) {
    lower.x = std::min(lower.x, point.x);
    lower.y = std::min(lower.y, point.y);
    lower.z = std::min(lower.z, point.z);
    upper.x = std::max(upper.x, point.x);
    upper.y = std::max(upper.y, point.y);
    upper.z = std::max(upper.z, point.z);
}

template <typename T>
void Aabb3<T>::expand(const Aabb3<T>& other) {
    if (!other.is_valid()) {
        return;
    }
    expand(other.lower);
    expand(other.upper);
}

template <typename T>
constexpr bool Aabb3<T>::overlaps(const Aabb3<T>& other) const {
    if (!is_valid() || !other.is_valid()) {
        return false;
    }
    return lower.x <= other.upper.x && upper.x >= other.lower.x && lower.y <= other.upper.y &&
           upper.y >= other.lower.y && lower.z <= other.upper.z && upper.z >= other.lower.z;
}

namespace detail {

template <typename T>
constexpr T get_axis_value(const Vec3<T>& value, int axis) {
    if (axis == 0) {
        return value.x;
    }
    if (axis == 1) {
        return value.y;
    }
    return value.z;
}

template <typename T>
double dot(const Vec3<T>& left, const Vec3<T>& right) {
    return static_cast<double>(left.x) * static_cast<double>(right.x) +
           static_cast<double>(left.y) * static_cast<double>(right.y) +
           static_cast<double>(left.z) * static_cast<double>(right.z);
}

template <typename T>
double length(const Vec3<T>& value) {
    return std::sqrt(dot(value, value));
}

template <typename T>
Vec3<double> to_double(const Vec3<T>& value) {
    return Vec3<double>(static_cast<double>(value.x), static_cast<double>(value.y), static_cast<double>(value.z));
}

template <typename T>
Vec3<double> cross(const Vec3<T>& left, const Vec3<T>& right) {
    return Vec3<double>(static_cast<double>(left.y) * static_cast<double>(right.z) -
                            static_cast<double>(left.z) * static_cast<double>(right.y),
                        static_cast<double>(left.z) * static_cast<double>(right.x) -
                            static_cast<double>(left.x) * static_cast<double>(right.z),
                        static_cast<double>(left.x) * static_cast<double>(right.y) -
                            static_cast<double>(left.y) * static_cast<double>(right.x));
}

template <typename T>
double squared_length(const Vec3<T>& value) {
    return dot(value, value);
}

template <typename T>
Aabb3<T> compute_triangle_bounds(const Triangle3<T>& triangle) {
    Aabb3<T> bounds = Aabb3<T>::empty();
    bounds.expand(triangle.a);
    bounds.expand(triangle.b);
    bounds.expand(triangle.c);
    return bounds;
}

template <typename T>
Aabb3<T> compute_quad_bounds(const Quad3<T>& quad) {
    Aabb3<T> bounds = Aabb3<T>::empty();
    bounds.expand(quad.a);
    bounds.expand(quad.b);
    bounds.expand(quad.c);
    bounds.expand(quad.d);
    return bounds;
}

template <typename T>
double point_aabb_distance_squared(const Vec3<double>& point, const Aabb3<T>& bounds) {
    if (!bounds.is_valid()) {
        return std::numeric_limits<double>::infinity();
    }

    double distance_squared = 0.0;
    const double lower_x = static_cast<double>(bounds.lower.x);
    const double lower_y = static_cast<double>(bounds.lower.y);
    const double lower_z = static_cast<double>(bounds.lower.z);
    const double upper_x = static_cast<double>(bounds.upper.x);
    const double upper_y = static_cast<double>(bounds.upper.y);
    const double upper_z = static_cast<double>(bounds.upper.z);

    if (point.x < lower_x) {
        const double delta = lower_x - point.x;
        distance_squared += delta * delta;
    } else if (point.x > upper_x) {
        const double delta = point.x - upper_x;
        distance_squared += delta * delta;
    }

    if (point.y < lower_y) {
        const double delta = lower_y - point.y;
        distance_squared += delta * delta;
    } else if (point.y > upper_y) {
        const double delta = point.y - upper_y;
        distance_squared += delta * delta;
    }

    if (point.z < lower_z) {
        const double delta = lower_z - point.z;
        distance_squared += delta * delta;
    } else if (point.z > upper_z) {
        const double delta = point.z - upper_z;
        distance_squared += delta * delta;
    }

    return distance_squared;
}

namespace {

inline Vec3<double> closest_point_on_segment(const Vec3<double>& point,
                                             const Vec3<double>& start,
                                             const Vec3<double>& end) {
    const Vec3<double> segment = end - start;
    const double length_squared = squared_length(segment);
    if (length_squared <= 0.0) {
        return start;
    }

    const double parameter = std::max(0.0, std::min(1.0, dot(point - start, segment) / length_squared));
    return start + segment * parameter;
}

inline Vec3<double> closest_point_on_degenerate_triangle(const Vec3<double>& point,
                                                         const Vec3<double>& a,
                                                         const Vec3<double>& b,
                                                         const Vec3<double>& c) {
    const std::array<Vec3<double>, 3> candidates{
        closest_point_on_segment(point, a, b),
        closest_point_on_segment(point, b, c),
        closest_point_on_segment(point, c, a)};

    Vec3<double> closest = candidates[0];
    double best_distance_squared = squared_length(point - closest);
    for (std::size_t index = 1; index < candidates.size(); ++index) {
        const double candidate_distance_squared = squared_length(point - candidates[index]);
        if (candidate_distance_squared < best_distance_squared) {
            closest = candidates[index];
            best_distance_squared = candidate_distance_squared;
        }
    }

    return closest;
}

}  // namespace

template <typename T>
Vec3<double> compute_closest_point_on_triangle(const Vec3<double>& point, const Triangle3<T>& triangle) {
    const Vec3<double> a = to_double(triangle.a);
    const Vec3<double> b = to_double(triangle.b);
    const Vec3<double> c = to_double(triangle.c);
    const Vec3<double> ab = b - a;
    const Vec3<double> ac = c - a;
    const Vec3<double> bc = c - b;

    const double ab_length_squared = squared_length(ab);
    const double ac_length_squared = squared_length(ac);
    const double bc_length_squared = squared_length(bc);
    const double max_edge_length_squared = std::max({ab_length_squared, ac_length_squared, bc_length_squared});
    if (max_edge_length_squared <= 0.0) {
        return a;
    }

    const double normal_length_squared = squared_length(cross(ab, ac));
    if (normal_length_squared <=
        std::numeric_limits<double>::epsilon() * max_edge_length_squared * max_edge_length_squared) {
        return closest_point_on_degenerate_triangle(point, a, b, c);
    }

    const Vec3<double> ap = point - a;
    const double d1 = dot(ab, ap);
    const double d2 = dot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        return a;
    }

    const Vec3<double> bp = point - b;
    const double d3 = dot(ab, bp);
    const double d4 = dot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) {
        return b;
    }

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);
        return a + ab * v;
    }

    const Vec3<double> cp = point - c;
    const double d5 = dot(ab, cp);
    const double d6 = dot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) {
        return c;
    }

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double w = d2 / (d2 - d6);
        return a + ac * w;
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && d4 >= d3 && d5 >= d6) {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + bc * w;
    }

    const double denominator = va + vb + vc;
    if (std::abs(denominator) <= 0.0) {
        return closest_point_on_degenerate_triangle(point, a, b, c);
    }

    const double v = vb / denominator;
    const double w = vc / denominator;
    return a + ab * v + ac * w;
}

template <typename T>
Vec3<double> compute_triangle_centroid(const Triangle3<T>& triangle) {
    return Vec3<double>((static_cast<double>(triangle.a.x) + static_cast<double>(triangle.b.x) +
                         static_cast<double>(triangle.c.x)) /
                            3.0,
                        (static_cast<double>(triangle.a.y) + static_cast<double>(triangle.b.y) +
                         static_cast<double>(triangle.c.y)) /
                            3.0,
                        (static_cast<double>(triangle.a.z) + static_cast<double>(triangle.b.z) +
                         static_cast<double>(triangle.c.z)) /
                            3.0);
}

template <typename T>
double compute_corner_angle(const Vec3<T>& left, const Vec3<T>& vertex, const Vec3<T>& right) {
    const Vec3<T> left_edge = left - vertex;
    const Vec3<T> right_edge = right - vertex;
    const double left_length = length(left_edge);
    const double right_length = length(right_edge);
    if (left_length == 0.0 || right_length == 0.0) {
        return 0.0;
    }
    double cosine_value = dot(left_edge, right_edge) / (left_length * right_length);
    cosine_value = std::max(-1.0, std::min(1.0, cosine_value));
    return std::acos(cosine_value) * 180.0 / 3.14159265358979323846;
}

template <typename T>
double compute_minimum_triangle_angle(const Triangle3<T>& triangle) {
    const double angle_a = compute_corner_angle(triangle.b, triangle.a, triangle.c);
    const double angle_b = compute_corner_angle(triangle.a, triangle.b, triangle.c);
    const double angle_c = compute_corner_angle(triangle.a, triangle.c, triangle.b);
    return std::min(angle_a, std::min(angle_b, angle_c));
}

namespace {

constexpr double kIntersectionEpsilonFactor = 1e-9;

struct Vec2d {
    double x = 0.0;
    double y = 0.0;
};

enum class TriangleShape {
    kTriangle,
    kSegment,
    kPoint,
};

template <typename T>
double max_triangle_edge_length(const Triangle3<T>& triangle) {
    return std::max({length(triangle.b - triangle.a), length(triangle.c - triangle.b), length(triangle.a - triangle.c)});
}

template <typename T>
double compute_intersection_epsilon(const Triangle3<T>& left, const Triangle3<T>& right) {
    return kIntersectionEpsilonFactor * std::max(1.0, std::max(max_triangle_edge_length(left), max_triangle_edge_length(right)));
}

template <typename T>
Vec3<double> compute_triangle_normal(const Triangle3<T>& triangle) {
    return cross(triangle.b - triangle.a, triangle.c - triangle.a);
}

template <typename T>
Vec3<double> compute_unit_triangle_normal(const Triangle3<T>& triangle) {
    const Vec3<double> normal = compute_triangle_normal(triangle);
    const double normal_length = length(normal);
    if (normal_length == 0.0) {
        return Vec3<double>(0.0, 0.0, 0.0);
    }
    return normal / normal_length;
}

inline int choose_projection_axis_from_normal(const Vec3<double>& normal) {
    const double x_abs = std::abs(normal.x);
    const double y_abs = std::abs(normal.y);
    const double z_abs = std::abs(normal.z);
    if (x_abs >= y_abs && x_abs >= z_abs) {
        return 0;
    }
    if (y_abs >= z_abs) {
        return 1;
    }
    return 2;
}

template <typename T>
Vec2d project_point_to_2d(const Vec3<T>& point, int axis) {
    if (axis == 0) {
        return Vec2d{static_cast<double>(point.y), static_cast<double>(point.z)};
    }
    if (axis == 1) {
        return Vec2d{static_cast<double>(point.x), static_cast<double>(point.z)};
    }
    return Vec2d{static_cast<double>(point.x), static_cast<double>(point.y)};
}

inline double orient2d(const Vec2d& a, const Vec2d& b, const Vec2d& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

inline bool point_in_2d_bounds(const Vec2d& point, const Vec2d& a, const Vec2d& b, double epsilon) {
    return point.x >= std::min(a.x, b.x) - epsilon && point.x <= std::max(a.x, b.x) + epsilon &&
           point.y >= std::min(a.y, b.y) - epsilon && point.y <= std::max(a.y, b.y) + epsilon;
}

inline bool point_on_segment_2d(const Vec2d& point, const Vec2d& a, const Vec2d& b, double epsilon) {
    return std::abs(orient2d(a, b, point)) <= epsilon && point_in_2d_bounds(point, a, b, epsilon);
}

inline bool segments_intersect_2d(const Vec2d& a0,
                                  const Vec2d& a1,
                                  const Vec2d& b0,
                                  const Vec2d& b1,
                                  double epsilon) {
    const double o0 = orient2d(a0, a1, b0);
    const double o1 = orient2d(a0, a1, b1);
    const double o2 = orient2d(b0, b1, a0);
    const double o3 = orient2d(b0, b1, a1);

    const bool proper_intersection =
        ((o0 > epsilon && o1 < -epsilon) || (o0 < -epsilon && o1 > epsilon)) &&
        ((o2 > epsilon && o3 < -epsilon) || (o2 < -epsilon && o3 > epsilon));
    if (proper_intersection) {
        return true;
    }

    return point_on_segment_2d(b0, a0, a1, epsilon) || point_on_segment_2d(b1, a0, a1, epsilon) ||
           point_on_segment_2d(a0, b0, b1, epsilon) || point_on_segment_2d(a1, b0, b1, epsilon);
}

inline bool point_in_triangle_2d(const Vec2d& point,
                                 const Vec2d& a,
                                 const Vec2d& b,
                                 const Vec2d& c,
                                 double epsilon) {
    const double o0 = orient2d(a, b, point);
    const double o1 = orient2d(b, c, point);
    const double o2 = orient2d(c, a, point);
    const bool non_negative = o0 >= -epsilon && o1 >= -epsilon && o2 >= -epsilon;
    const bool non_positive = o0 <= epsilon && o1 <= epsilon && o2 <= epsilon;
    return non_negative || non_positive;
}

template <typename T>
TriangleShape classify_triangle_shape(const Triangle3<T>& triangle,
                                      double epsilon,
                                      std::array<Vec3<double>, 2>& segment,
                                      Vec3<double>& point) {
    const Vec3<double> a = to_double(triangle.a);
    const Vec3<double> b = to_double(triangle.b);
    const Vec3<double> c = to_double(triangle.c);

    const double ab_length_squared = squared_length(b - a);
    const double bc_length_squared = squared_length(c - b);
    const double ca_length_squared = squared_length(a - c);
    const double max_edge_length_squared = std::max({ab_length_squared, bc_length_squared, ca_length_squared});
    if (max_edge_length_squared <= epsilon * epsilon) {
        point = a;
        segment = {a, a};
        return TriangleShape::kPoint;
    }

    if (ab_length_squared >= bc_length_squared && ab_length_squared >= ca_length_squared) {
        segment = {a, b};
        point = c;
    } else if (bc_length_squared >= ca_length_squared) {
        segment = {b, c};
        point = a;
    } else {
        segment = {c, a};
        point = b;
    }

    const Vec3<double> normal = compute_triangle_normal(triangle);
    if (squared_length(normal) <= epsilon * epsilon * max_edge_length_squared) {
        return TriangleShape::kSegment;
    }

    point = a;
    return TriangleShape::kTriangle;
}

template <typename T>
double signed_distance_to_plane(const Vec3<T>& point, const Vec3<T>& plane_point, const Vec3<double>& unit_normal) {
    return dot(to_double(point) - to_double(plane_point), unit_normal);
}

inline bool point_on_segment_3d(const Vec3<double>& point,
                                const Vec3<double>& start,
                                const Vec3<double>& end,
                                double epsilon) {
    const Vec3<double> segment = end - start;
    const double segment_length_squared = squared_length(segment);
    if (segment_length_squared <= epsilon * epsilon) {
        return squared_length(point - start) <= epsilon * epsilon;
    }

    const Vec3<double> point_offset = point - start;
    if (squared_length(cross(point_offset, segment)) > epsilon * epsilon * segment_length_squared) {
        return false;
    }

    const double parameter = dot(point_offset, segment) / segment_length_squared;
    if (parameter < -epsilon || parameter > 1.0 + epsilon) {
        return false;
    }

    const Vec3<double> closest_point = start + segment * std::max(0.0, std::min(1.0, parameter));
    return squared_length(point - closest_point) <= epsilon * epsilon;
}

inline double segment_segment_distance_squared(const Vec3<double>& left_start,
                                               const Vec3<double>& left_end,
                                               const Vec3<double>& right_start,
                                               const Vec3<double>& right_end) {
    const Vec3<double> left_direction = left_end - left_start;
    const Vec3<double> right_direction = right_end - right_start;
    const Vec3<double> start_offset = left_start - right_start;

    const double left_length_squared = squared_length(left_direction);
    const double right_length_squared = squared_length(right_direction);
    const double cross_term = dot(left_direction, right_direction);
    const double left_offset = dot(left_direction, start_offset);
    const double right_offset = dot(right_direction, start_offset);

    double left_parameter = 0.0;
    double right_parameter = 0.0;

    if (left_length_squared <= 0.0 && right_length_squared <= 0.0) {
        return squared_length(left_start - right_start);
    }

    if (left_length_squared <= 0.0) {
        right_parameter = std::max(0.0, std::min(1.0, right_offset / right_length_squared));
    } else if (right_length_squared <= 0.0) {
        left_parameter = std::max(0.0, std::min(1.0, -left_offset / left_length_squared));
    } else {
        const double denominator = left_length_squared * right_length_squared - cross_term * cross_term;
        if (denominator != 0.0) {
            left_parameter = std::max(
                0.0,
                std::min(1.0, (cross_term * right_offset - left_offset * right_length_squared) / denominator));
        }

        right_parameter = (cross_term * left_parameter + right_offset) / right_length_squared;
        if (right_parameter < 0.0) {
            right_parameter = 0.0;
            left_parameter = std::max(0.0, std::min(1.0, -left_offset / left_length_squared));
        } else if (right_parameter > 1.0) {
            right_parameter = 1.0;
            left_parameter = std::max(0.0, std::min(1.0, (cross_term - left_offset) / left_length_squared));
        }
    }

    const Vec3<double> left_closest = left_start + left_direction * left_parameter;
    const Vec3<double> right_closest = right_start + right_direction * right_parameter;
    return squared_length(left_closest - right_closest);
}

template <typename T>
bool point_in_triangle_on_plane(const Vec3<double>& point,
                                const Triangle3<T>& triangle,
                                const Vec3<double>& plane_normal,
                                double epsilon) {
    const int projection_axis = choose_projection_axis_from_normal(plane_normal);
    const Vec2d projected_point = project_point_to_2d(point, projection_axis);
    const Vec2d projected_a = project_point_to_2d(triangle.a, projection_axis);
    const Vec2d projected_b = project_point_to_2d(triangle.b, projection_axis);
    const Vec2d projected_c = project_point_to_2d(triangle.c, projection_axis);
    return point_in_triangle_2d(projected_point, projected_a, projected_b, projected_c, epsilon);
}

template <typename T>
bool coplanar_segment_triangle_intersect(const Vec3<double>& segment_start,
                                         const Vec3<double>& segment_end,
                                         const Triangle3<T>& triangle,
                                         const Vec3<double>& plane_normal,
                                         double epsilon) {
    const int projection_axis = choose_projection_axis_from_normal(plane_normal);
    const Vec2d projected_segment_start = project_point_to_2d(segment_start, projection_axis);
    const Vec2d projected_segment_end = project_point_to_2d(segment_end, projection_axis);
    const Vec2d projected_a = project_point_to_2d(triangle.a, projection_axis);
    const Vec2d projected_b = project_point_to_2d(triangle.b, projection_axis);
    const Vec2d projected_c = project_point_to_2d(triangle.c, projection_axis);

    if (point_in_triangle_2d(projected_segment_start, projected_a, projected_b, projected_c, epsilon) ||
        point_in_triangle_2d(projected_segment_end, projected_a, projected_b, projected_c, epsilon)) {
        return true;
    }

    return segments_intersect_2d(projected_segment_start, projected_segment_end, projected_a, projected_b, epsilon) ||
           segments_intersect_2d(projected_segment_start, projected_segment_end, projected_b, projected_c, epsilon) ||
           segments_intersect_2d(projected_segment_start, projected_segment_end, projected_c, projected_a, epsilon);
}

template <typename T>
bool coplanar_triangles_intersect(const Triangle3<T>& left,
                                  const Triangle3<T>& right,
                                  const Vec3<double>& plane_normal,
                                  double epsilon) {
    const int projection_axis = choose_projection_axis_from_normal(plane_normal);
    const std::array<Vec2d, 3> left_vertices{
        project_point_to_2d(left.a, projection_axis),
        project_point_to_2d(left.b, projection_axis),
        project_point_to_2d(left.c, projection_axis)};
    const std::array<Vec2d, 3> right_vertices{
        project_point_to_2d(right.a, projection_axis),
        project_point_to_2d(right.b, projection_axis),
        project_point_to_2d(right.c, projection_axis)};

    for (std::size_t left_index = 0; left_index < 3U; ++left_index) {
        const std::size_t left_next = (left_index + 1U) % 3U;
        for (std::size_t right_index = 0; right_index < 3U; ++right_index) {
            const std::size_t right_next = (right_index + 1U) % 3U;
            if (segments_intersect_2d(left_vertices[left_index],
                                      left_vertices[left_next],
                                      right_vertices[right_index],
                                      right_vertices[right_next],
                                      epsilon)) {
                return true;
            }
        }
    }

    return point_in_triangle_2d(left_vertices[0], right_vertices[0], right_vertices[1], right_vertices[2], epsilon) ||
           point_in_triangle_2d(right_vertices[0], left_vertices[0], left_vertices[1], left_vertices[2], epsilon);
}

template <typename T>
bool point_intersects_triangle(const Vec3<double>& point, const Triangle3<T>& triangle, double epsilon);

template <typename T>
bool segment_intersects_triangle(const Vec3<double>& segment_start,
                                 const Vec3<double>& segment_end,
                                 const Triangle3<T>& triangle,
                                 double epsilon);

template <typename T>
bool point_intersects_triangle(const Vec3<double>& point, const Triangle3<T>& triangle, double epsilon) {
    std::array<Vec3<double>, 2> segment{};
    Vec3<double> reference_point;
    const TriangleShape shape = classify_triangle_shape(triangle, epsilon, segment, reference_point);
    if (shape == TriangleShape::kPoint) {
        return squared_length(point - segment[0]) <= epsilon * epsilon;
    }
    if (shape == TriangleShape::kSegment) {
        return point_on_segment_3d(point, segment[0], segment[1], epsilon);
    }

    const Vec3<double> plane_normal = compute_unit_triangle_normal(triangle);
    if (std::abs(dot(point - to_double(triangle.a), plane_normal)) > epsilon) {
        return false;
    }
    return point_in_triangle_on_plane(point, triangle, plane_normal, epsilon);
}

template <typename T>
bool segment_intersects_triangle(const Vec3<double>& segment_start,
                                 const Vec3<double>& segment_end,
                                 const Triangle3<T>& triangle,
                                 double epsilon) {
    std::array<Vec3<double>, 2> reduced_segment{};
    Vec3<double> reference_point;
    const TriangleShape shape = classify_triangle_shape(triangle, epsilon, reduced_segment, reference_point);
    if (shape == TriangleShape::kPoint) {
        return point_on_segment_3d(reduced_segment[0], segment_start, segment_end, epsilon);
    }
    if (shape == TriangleShape::kSegment) {
        return segment_segment_distance_squared(segment_start, segment_end, reduced_segment[0], reduced_segment[1]) <=
               epsilon * epsilon;
    }

    const Vec3<double> plane_normal = compute_unit_triangle_normal(triangle);
    const double start_distance = dot(segment_start - to_double(triangle.a), plane_normal);
    const double end_distance = dot(segment_end - to_double(triangle.a), plane_normal);

    if ((start_distance > epsilon && end_distance > epsilon) || (start_distance < -epsilon && end_distance < -epsilon)) {
        return false;
    }

    if (std::abs(start_distance) <= epsilon && std::abs(end_distance) <= epsilon) {
        return coplanar_segment_triangle_intersect(segment_start, segment_end, triangle, plane_normal, epsilon);
    }

    if (std::abs(start_distance) <= epsilon && point_in_triangle_on_plane(segment_start, triangle, plane_normal, epsilon)) {
        return true;
    }
    if (std::abs(end_distance) <= epsilon && point_in_triangle_on_plane(segment_end, triangle, plane_normal, epsilon)) {
        return true;
    }

    const double denominator = start_distance - end_distance;
    if (std::abs(denominator) <= epsilon) {
        return false;
    }

    const double parameter = start_distance / denominator;
    if (parameter < -epsilon || parameter > 1.0 + epsilon) {
        return false;
    }

    const double clamped_parameter = std::max(0.0, std::min(1.0, parameter));
    const Vec3<double> intersection_point =
        segment_start + (segment_end - segment_start) * clamped_parameter;
    return point_in_triangle_on_plane(intersection_point, triangle, plane_normal, epsilon);
}

}  // namespace

template <typename T>
bool triangles_intersect(const Triangle3<T>& left, const Triangle3<T>& right) {
    if (!compute_triangle_bounds(left).overlaps(compute_triangle_bounds(right))) {
        return false;
    }

    const double epsilon = compute_intersection_epsilon(left, right);

    std::array<Vec3<double>, 2> left_segment{};
    std::array<Vec3<double>, 2> right_segment{};
    Vec3<double> left_point;
    Vec3<double> right_point;
    const TriangleShape left_shape = classify_triangle_shape(left, epsilon, left_segment, left_point);
    const TriangleShape right_shape = classify_triangle_shape(right, epsilon, right_segment, right_point);

    if (left_shape == TriangleShape::kPoint && right_shape == TriangleShape::kPoint) {
        return squared_length(left_segment[0] - right_segment[0]) <= epsilon * epsilon;
    }
    if (left_shape == TriangleShape::kPoint && right_shape == TriangleShape::kSegment) {
        return point_on_segment_3d(left_segment[0], right_segment[0], right_segment[1], epsilon);
    }
    if (left_shape == TriangleShape::kSegment && right_shape == TriangleShape::kPoint) {
        return point_on_segment_3d(right_segment[0], left_segment[0], left_segment[1], epsilon);
    }
    if (left_shape == TriangleShape::kPoint) {
        return point_intersects_triangle(left_segment[0], right, epsilon);
    }
    if (right_shape == TriangleShape::kPoint) {
        return point_intersects_triangle(right_segment[0], left, epsilon);
    }
    if (left_shape == TriangleShape::kSegment && right_shape == TriangleShape::kSegment) {
        return segment_segment_distance_squared(left_segment[0], left_segment[1], right_segment[0], right_segment[1]) <=
               epsilon * epsilon;
    }
    if (left_shape == TriangleShape::kSegment) {
        return segment_intersects_triangle(left_segment[0], left_segment[1], right, epsilon);
    }
    if (right_shape == TriangleShape::kSegment) {
        return segment_intersects_triangle(right_segment[0], right_segment[1], left, epsilon);
    }

    const std::array<Vec3<double>, 3> left_vertices{to_double(left.a), to_double(left.b), to_double(left.c)};
    const std::array<Vec3<double>, 3> right_vertices{to_double(right.a), to_double(right.b), to_double(right.c)};

    for (std::size_t index = 0; index < 3U; ++index) {
        const std::size_t next = (index + 1U) % 3U;
        if (segment_intersects_triangle(left_vertices[index], left_vertices[next], right, epsilon) ||
            segment_intersects_triangle(right_vertices[index], right_vertices[next], left, epsilon)) {
            return true;
        }
    }

    return point_intersects_triangle(left_vertices[0], right, epsilon) ||
           point_intersects_triangle(right_vertices[0], left, epsilon);
}

}  // namespace detail

template <typename T>
QuadTriangulation<T> split_quad_maximizing_minimum_angle(const Quad3<T>& quad) {
    const Triangle3<T> diagonal_ac_first{quad.a, quad.b, quad.c};
    const Triangle3<T> diagonal_ac_second{quad.a, quad.c, quad.d};
    const Triangle3<T> diagonal_bd_first{quad.a, quad.b, quad.d};
    const Triangle3<T> diagonal_bd_second{quad.b, quad.c, quad.d};

    const double diagonal_ac_score =
        std::min(detail::compute_minimum_triangle_angle(diagonal_ac_first),
                 detail::compute_minimum_triangle_angle(diagonal_ac_second));
    const double diagonal_bd_score =
        std::min(detail::compute_minimum_triangle_angle(diagonal_bd_first),
                 detail::compute_minimum_triangle_angle(diagonal_bd_second));

    if (diagonal_bd_score > diagonal_ac_score) {
        return QuadTriangulation<T>{diagonal_bd_first, diagonal_bd_second, {{1U, 3U}}, diagonal_bd_score};
    }

    return QuadTriangulation<T>{diagonal_ac_first, diagonal_ac_second, {{0U, 2U}}, diagonal_ac_score};
}

template <typename T>
template <typename Callback>
void PrimitiveAdapter<Triangle3<T>>::for_each_triangle(const Triangle3<T>& primitive, Callback&& callback) {
    callback(primitive);
}

template <typename T>
template <typename Callback>
void PrimitiveAdapter<Quad3<T>>::for_each_triangle(const Quad3<T>& primitive, Callback&& callback) {
    const QuadTriangulation<T> triangulation = split_quad_maximizing_minimum_angle(primitive);
    callback(triangulation.first_triangle);
    callback(triangulation.second_triangle);
}

template <typename... Primitives>
template <typename Callback>
void PrimitiveAdapter<std::variant<Primitives...>>::for_each_triangle(const std::variant<Primitives...>& primitive,
                                                                      Callback&& callback) {
    std::visit(
        [&](const auto& value) {
            using active_type = std::decay_t<decltype(value)>;
            PrimitiveAdapter<active_type>::for_each_triangle(value, std::forward<Callback>(callback));
        },
        primitive);
}

template <typename Primitive, typename Adapter>
Bvh3<Primitive, Adapter>::Bvh3(std::size_t leaf_size) : leaf_size_(std::max<std::size_t>(1, leaf_size)) {}

template <typename Primitive, typename Adapter>
template <typename InputIt>
void Bvh3<Primitive, Adapter>::build(InputIt first, InputIt last) {
    clear();

    std::size_t primitive_index = 0;
    for (InputIt it = first; it != last; ++it, ++primitive_index) {
        std::size_t local_triangle_index = 0;
        Adapter::for_each_triangle(*it, [&](const Triangle3<scalar_type>& triangle) {
            triangles_.push_back(make_triangle_record(triangle, primitive_index, local_triangle_index));
            ++local_triangle_index;
        });
    }

    primitive_count_ = primitive_index;
    if (triangles_.empty()) {
        return;
    }

    nodes_.reserve(triangles_.size() * 2U);
    root_node_index_ = build_node(0, triangles_.size());
}

template <typename Primitive, typename Adapter>
void Bvh3<Primitive, Adapter>::build(const std::vector<Primitive>& primitives) {
    build(primitives.begin(), primitives.end());
}

template <typename Primitive, typename Adapter>
bool Bvh3<Primitive, Adapter>::empty() const {
    return triangles_.empty();
}

template <typename Primitive, typename Adapter>
std::size_t Bvh3<Primitive, Adapter>::primitive_count() const {
    return primitive_count_;
}

template <typename Primitive, typename Adapter>
std::size_t Bvh3<Primitive, Adapter>::triangle_count() const {
    return triangles_.size();
}

template <typename Primitive, typename Adapter>
std::vector<const typename Bvh3<Primitive, Adapter>::TriangleRecord*>
Bvh3<Primitive, Adapter>::collect_overlapping_triangles(const Aabb3<scalar_type>& query_bounds) const {
    std::vector<const TriangleRecord*> hits;
    visit_overlapping_triangles(query_bounds, [&](const TriangleRecord& record) { hits.push_back(&record); });
    return hits;
}

template <typename Primitive, typename Adapter>
std::vector<std::size_t> Bvh3<Primitive, Adapter>::collect_overlapping_primitive_indices(
    const Aabb3<scalar_type>& query_bounds) const {
    std::vector<std::size_t> primitive_indices;
    visit_overlapping_triangles(query_bounds, [&](const TriangleRecord& record) {
        primitive_indices.push_back(record.primitive_index);
    });
    std::sort(primitive_indices.begin(), primitive_indices.end());
    primitive_indices.erase(std::unique(primitive_indices.begin(), primitive_indices.end()), primitive_indices.end());
    return primitive_indices;
}

template <typename Primitive, typename Adapter>
std::vector<const typename Bvh3<Primitive, Adapter>::TriangleRecord*>
Bvh3<Primitive, Adapter>::collect_intersecting_triangles(const Triangle3<scalar_type>& query_triangle) const {
    std::vector<const TriangleRecord*> hits;
    visit_intersecting_triangles(query_triangle, [&](const TriangleRecord& record) { hits.push_back(&record); });
    return hits;
}

template <typename Primitive, typename Adapter>
std::vector<const typename Bvh3<Primitive, Adapter>::TriangleRecord*>
Bvh3<Primitive, Adapter>::collect_intersecting_triangles(const Quad3<scalar_type>& query_quad) const {
    std::vector<const TriangleRecord*> hits;
    visit_intersecting_triangles(query_quad, [&](const TriangleRecord& record) { hits.push_back(&record); });
    return hits;
}

template <typename Primitive, typename Adapter>
std::vector<std::size_t> Bvh3<Primitive, Adapter>::collect_intersecting_primitive_indices(
    const Triangle3<scalar_type>& query_triangle) const {
    std::vector<std::size_t> primitive_indices;
    visit_intersecting_triangles(query_triangle, [&](const TriangleRecord& record) {
        primitive_indices.push_back(record.primitive_index);
    });
    std::sort(primitive_indices.begin(), primitive_indices.end());
    primitive_indices.erase(std::unique(primitive_indices.begin(), primitive_indices.end()), primitive_indices.end());
    return primitive_indices;
}

template <typename Primitive, typename Adapter>
std::vector<std::size_t> Bvh3<Primitive, Adapter>::collect_intersecting_primitive_indices(
    const Quad3<scalar_type>& query_quad) const {
    std::vector<std::size_t> primitive_indices;
    visit_intersecting_triangles(query_quad, [&](const TriangleRecord& record) {
        primitive_indices.push_back(record.primitive_index);
    });
    std::sort(primitive_indices.begin(), primitive_indices.end());
    primitive_indices.erase(std::unique(primitive_indices.begin(), primitive_indices.end()), primitive_indices.end());
    return primitive_indices;
}

template <typename Primitive, typename Adapter>
std::optional<typename Bvh3<Primitive, Adapter>::NearestFaceResult> Bvh3<Primitive, Adapter>::find_nearest_face(
    const Vec3<scalar_type>& point) const {
    if (root_node_index_ == invalid_node_index) {
        return std::nullopt;
    }

    const Vec3<double> query_point = detail::to_double(point);
    NearestFaceResult best;

    std::vector<std::size_t> stack;
    stack.push_back(root_node_index_);
    while (!stack.empty()) {
        const std::size_t node_index = stack.back();
        stack.pop_back();

        const Node& node = nodes_[node_index];
        if (detail::point_aabb_distance_squared(query_point, node.bounds) > best.distance_squared) {
            continue;
        }

        if (node.is_leaf()) {
            for (std::size_t triangle_index = node.first_triangle;
                 triangle_index < node.first_triangle + node.triangle_count;
                 ++triangle_index) {
                const TriangleRecord& record = triangles_[triangle_index];
                if (detail::point_aabb_distance_squared(query_point, record.bounds) > best.distance_squared) {
                    continue;
                }

                const Vec3<double> closest_point =
                    detail::compute_closest_point_on_triangle(query_point, record.triangle);
                const Vec3<double> displacement = closest_point - query_point;
                const double distance_squared = detail::squared_length(displacement);
                if (distance_squared < best.distance_squared) {
                    best.triangle_record = &record;
                    best.closest_point = closest_point;
                    best.displacement = displacement;
                    best.distance_squared = distance_squared;
                    best.distance = std::sqrt(distance_squared);
                }
            }
            continue;
        }

        const double left_distance_squared =
            detail::point_aabb_distance_squared(query_point, nodes_[node.left_child].bounds);
        const double right_distance_squared =
            detail::point_aabb_distance_squared(query_point, nodes_[node.right_child].bounds);

        if (left_distance_squared < right_distance_squared) {
            if (right_distance_squared <= best.distance_squared) {
                stack.push_back(node.right_child);
            }
            if (left_distance_squared <= best.distance_squared) {
                stack.push_back(node.left_child);
            }
        } else {
            if (left_distance_squared <= best.distance_squared) {
                stack.push_back(node.left_child);
            }
            if (right_distance_squared <= best.distance_squared) {
                stack.push_back(node.right_child);
            }
        }
    }

    if (best.triangle_record == nullptr) {
        return std::nullopt;
    }
    return best;
}

template <typename Primitive, typename Adapter>
template <typename Callback>
void Bvh3<Primitive, Adapter>::visit_overlapping_triangles(const Aabb3<scalar_type>& query_bounds,
                                                           Callback&& callback) const {
    visit_overlapping_triangle_records(query_bounds, std::forward<Callback>(callback));
}

template <typename Primitive, typename Adapter>
template <typename Callback>
void Bvh3<Primitive, Adapter>::visit_intersecting_triangles(const Triangle3<scalar_type>& query_triangle,
                                                            Callback&& callback) const {
    visit_intersecting_triangle_records(std::array<Triangle3<scalar_type>, 1U>{query_triangle},
                                        std::forward<Callback>(callback));
}

template <typename Primitive, typename Adapter>
template <typename Callback>
void Bvh3<Primitive, Adapter>::visit_intersecting_triangles(const Quad3<scalar_type>& query_quad,
                                                            Callback&& callback) const {
    const QuadTriangulation<scalar_type> triangulation = split_quad_maximizing_minimum_angle(query_quad);
    visit_intersecting_triangle_records(
        std::array<Triangle3<scalar_type>, 2U>{triangulation.first_triangle, triangulation.second_triangle},
        std::forward<Callback>(callback));
}

template <typename Primitive, typename Adapter>
template <typename Callback>
void Bvh3<Primitive, Adapter>::visit_overlapping_triangle_records(const Aabb3<scalar_type>& query_bounds,
                                                                  Callback&& callback) const {
    if (root_node_index_ == invalid_node_index) {
        return;
    }

    std::vector<std::size_t> stack;
    stack.push_back(root_node_index_);
    while (!stack.empty()) {
        const std::size_t node_index = stack.back();
        stack.pop_back();

        const Node& node = nodes_[node_index];
        if (!node.bounds.overlaps(query_bounds)) {
            continue;
        }

        if (node.is_leaf()) {
            for (std::size_t triangle_index = node.first_triangle;
                 triangle_index < node.first_triangle + node.triangle_count;
                 ++triangle_index) {
                const TriangleRecord& record = triangles_[triangle_index];
                if (record.bounds.overlaps(query_bounds)) {
                    callback(record);
                }
            }
            continue;
        }

        stack.push_back(node.left_child);
        stack.push_back(node.right_child);
    }
}

template <typename Primitive, typename Adapter>
template <std::size_t QueryTriangleCount, typename Callback>
void Bvh3<Primitive, Adapter>::visit_intersecting_triangle_records(
    const std::array<Triangle3<scalar_type>, QueryTriangleCount>& query_triangles,
    Callback&& callback) const {
    if (root_node_index_ == invalid_node_index) {
        return;
    }

    Aabb3<scalar_type> query_bounds = Aabb3<scalar_type>::empty();
    for (const Triangle3<scalar_type>& query_triangle : query_triangles) {
        query_bounds.expand(detail::compute_triangle_bounds(query_triangle));
    }

    visit_overlapping_triangle_records(query_bounds, [&](const TriangleRecord& record) {
        for (const Triangle3<scalar_type>& query_triangle : query_triangles) {
            if (detail::triangles_intersect(record.triangle, query_triangle)) {
                callback(record);
                break;
            }
        }
    });
}

template <typename Primitive, typename Adapter>
const std::vector<typename Bvh3<Primitive, Adapter>::TriangleRecord>& Bvh3<Primitive, Adapter>::triangles() const {
    return triangles_;
}

template <typename Primitive, typename Adapter>
bool Bvh3<Primitive, Adapter>::Node::is_leaf() const {
    return left_child == invalid_node_index && right_child == invalid_node_index;
}

template <typename Primitive, typename Adapter>
void Bvh3<Primitive, Adapter>::clear() {
    triangles_.clear();
    nodes_.clear();
    primitive_count_ = 0;
    root_node_index_ = invalid_node_index;
}

template <typename Primitive, typename Adapter>
typename Bvh3<Primitive, Adapter>::TriangleRecord Bvh3<Primitive, Adapter>::make_triangle_record(
    const Triangle3<scalar_type>& triangle,
    std::size_t primitive_index,
    std::size_t local_triangle_index) const {
    return TriangleRecord{triangle,
                          detail::compute_triangle_bounds(triangle),
                          detail::compute_triangle_centroid(triangle),
                          primitive_index,
                          local_triangle_index};
}

template <typename Primitive, typename Adapter>
Aabb3<typename Bvh3<Primitive, Adapter>::scalar_type> Bvh3<Primitive, Adapter>::compute_range_bounds(
    std::size_t begin,
    std::size_t end) const {
    Aabb3<scalar_type> bounds = Aabb3<scalar_type>::empty();
    for (std::size_t index = begin; index < end; ++index) {
        bounds.expand(triangles_[index].bounds);
    }
    return bounds;
}

template <typename Primitive, typename Adapter>
Aabb3<double> Bvh3<Primitive, Adapter>::compute_centroid_bounds(std::size_t begin, std::size_t end) const {
    Aabb3<double> centroid_bounds = Aabb3<double>::empty();
    for (std::size_t index = begin; index < end; ++index) {
        centroid_bounds.expand(triangles_[index].centroid);
    }
    return centroid_bounds;
}

template <typename Primitive, typename Adapter>
int Bvh3<Primitive, Adapter>::choose_split_axis(const Aabb3<double>& centroid_bounds) const {
    const double x_extent = centroid_bounds.upper.x - centroid_bounds.lower.x;
    const double y_extent = centroid_bounds.upper.y - centroid_bounds.lower.y;
    const double z_extent = centroid_bounds.upper.z - centroid_bounds.lower.z;

    if (x_extent <= 0.0 && y_extent <= 0.0 && z_extent <= 0.0) {
        return -1;
    }
    if (x_extent >= y_extent && x_extent >= z_extent) {
        return 0;
    }
    if (y_extent >= z_extent) {
        return 1;
    }
    return 2;
}

template <typename Primitive, typename Adapter>
std::size_t Bvh3<Primitive, Adapter>::build_node(std::size_t begin, std::size_t end) {
    const std::size_t node_index = nodes_.size();
    nodes_.push_back(Node{});
    nodes_[node_index].bounds = compute_range_bounds(begin, end);
    nodes_[node_index].first_triangle = begin;
    nodes_[node_index].triangle_count = end - begin;

    if (end - begin <= leaf_size_) {
        return node_index;
    }

    const Aabb3<double> centroid_bounds = compute_centroid_bounds(begin, end);
    const int split_axis = choose_split_axis(centroid_bounds);
    if (split_axis < 0) {
        return node_index;
    }

    const std::size_t middle = begin + (end - begin) / 2U;
    std::nth_element(triangles_.begin() + static_cast<std::ptrdiff_t>(begin),
                     triangles_.begin() + static_cast<std::ptrdiff_t>(middle),
                     triangles_.begin() + static_cast<std::ptrdiff_t>(end),
                     [&](const TriangleRecord& left, const TriangleRecord& right) {
                         return detail::get_axis_value(left.centroid, split_axis) <
                                detail::get_axis_value(right.centroid, split_axis);
                     });

    if (middle == begin || middle == end) {
        return node_index;
    }

    nodes_[node_index].left_child = build_node(begin, middle);
    nodes_[node_index].right_child = build_node(middle, end);
    return node_index;
}

}  // namespace bvh3d
}  // namespace zcode

#endif
