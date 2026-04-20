#ifndef ZCODE_BVH3D_BVH3D_H
#define ZCODE_BVH3D_BVH3D_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace zcode {
namespace bvh3d {

/// Represents a 3D vector or point.
template <typename T>
struct Vec3 {
    T x{};
    T y{};
    T z{};

    /// Builds a vector from three coordinates.
    constexpr Vec3();
    constexpr Vec3(T x_value, T y_value, T z_value);
};

template <typename T>
constexpr Vec3<T> operator+(const Vec3<T>& left, const Vec3<T>& right);

template <typename T>
constexpr Vec3<T> operator-(const Vec3<T>& left, const Vec3<T>& right);

template <typename T>
constexpr Vec3<T> operator*(const Vec3<T>& value, T scale);

template <typename T>
constexpr Vec3<T> operator/(const Vec3<T>& value, T scale);

/// Represents a closed axis-aligned bounding box.
template <typename T>
struct Aabb3 {
    Vec3<T> lower;
    Vec3<T> upper;

    /// Creates an invalid box that can be expanded incrementally.
    static constexpr Aabb3<T> empty();

    /// Returns true when the box has a valid lower/upper ordering.
    constexpr bool is_valid() const;

    /// Expands the box so that it contains the input point.
    void expand(const Vec3<T>& point);

    /// Expands the box so that it contains the input box.
    void expand(const Aabb3<T>& other);

    /// Returns true when two AABBs overlap or touch.
    constexpr bool overlaps(const Aabb3<T>& other) const;
};

/// Represents one normalized triangle primitive.
template <typename T>
struct Triangle3 {
    Vec3<T> a;
    Vec3<T> b;
    Vec3<T> c;
};

/// Represents one input quadrilateral primitive.
template <typename T>
struct Quad3 {
    Vec3<T> a;
    Vec3<T> b;
    Vec3<T> c;
    Vec3<T> d;
};

/// Stores the chosen triangulation for one quadrilateral.
template <typename T>
struct QuadTriangulation {
    Triangle3<T> first_triangle;
    Triangle3<T> second_triangle;
    std::array<std::size_t, 2> diagonal{{0U, 2U}};
    double minimum_angle_degrees = 0.0;
};

namespace detail {

template <typename T>
constexpr T get_axis_value(const Vec3<T>& value, int axis);

template <typename T>
double dot(const Vec3<T>& left, const Vec3<T>& right);

template <typename T>
double length(const Vec3<T>& value);

template <typename T>
Vec3<double> to_double(const Vec3<T>& value);

template <typename T>
Vec3<double> cross(const Vec3<T>& left, const Vec3<T>& right);

template <typename T>
double squared_length(const Vec3<T>& value);

template <typename T>
Aabb3<T> compute_triangle_bounds(const Triangle3<T>& triangle);

template <typename T>
Aabb3<T> compute_quad_bounds(const Quad3<T>& quad);

template <typename T>
Vec3<double> compute_triangle_centroid(const Triangle3<T>& triangle);

template <typename T>
double compute_corner_angle(const Vec3<T>& left, const Vec3<T>& vertex, const Vec3<T>& right);

template <typename T>
double compute_minimum_triangle_angle(const Triangle3<T>& triangle);

template <typename T>
bool triangles_intersect(const Triangle3<T>& left, const Triangle3<T>& right);

}  // namespace detail

/// Selects the quad diagonal that maximizes the minimum angle over the two triangles.
///
/// The criterion is:
/// max over diagonal d of min(theta_0, theta_1, ..., theta_5),
/// where theta_i enumerates all six interior angles produced by the split.
template <typename T>
QuadTriangulation<T> split_quad_maximizing_minimum_angle(const Quad3<T>& quad);

/// Defines how one primitive type is normalized into triangles for BVH construction.
template <typename Primitive>
struct PrimitiveAdapter;

template <typename T>
struct PrimitiveAdapter<Triangle3<T>> {
    using scalar_type = T;

    /// Emits exactly one triangle for a triangle primitive.
    template <typename Callback>
    static void for_each_triangle(const Triangle3<T>& primitive, Callback&& callback);
};

template <typename T>
struct PrimitiveAdapter<Quad3<T>> {
    using scalar_type = T;

    /// Emits the two triangles chosen by the quad split heuristic.
    template <typename Callback>
    static void for_each_triangle(const Quad3<T>& primitive, Callback&& callback);
};

template <typename... Primitives>
struct PrimitiveAdapter<std::variant<Primitives...>> {
    using scalar_type = std::common_type_t<typename PrimitiveAdapter<Primitives>::scalar_type...>;

    /// Dispatches to the adapter of the active variant alternative.
    template <typename Callback>
    static void for_each_triangle(const std::variant<Primitives...>& primitive, Callback&& callback);
};

/// Builds a 3D BVH over any primitive type that can be adapted to triangles.
template <typename Primitive, typename Adapter = PrimitiveAdapter<Primitive>>
class Bvh3 {
public:
    using primitive_type = Primitive;
    using adapter_type = Adapter;
    using scalar_type = typename Adapter::scalar_type;

    /// Stores one triangulated entry used by the BVH.
    struct TriangleRecord {
        Triangle3<scalar_type> triangle;
        Aabb3<scalar_type> bounds;
        Vec3<double> centroid;
        std::size_t primitive_index = 0;
        std::size_t local_triangle_index = 0;
    };

    /// Builds a BVH with a configurable leaf size.
    explicit Bvh3(std::size_t leaf_size = 4);

    /// Clears the current tree and rebuilds it from an input range.
    template <typename InputIt>
    void build(InputIt first, InputIt last);

    /// Convenience overload for vector-like storage.
    void build(const std::vector<Primitive>& primitives);

    /// Returns true when the BVH contains no triangles.
    bool empty() const;

    /// Returns the number of source primitives used to build the tree.
    std::size_t primitive_count() const;

    /// Returns the number of normalized triangles stored in the tree.
    std::size_t triangle_count() const;

    /// Returns all triangle hits whose AABBs overlap the query box.
    std::vector<const TriangleRecord*> collect_overlapping_triangles(const Aabb3<scalar_type>& query_bounds) const;

    /// Returns unique primitive indices whose triangulated bounds overlap the query box.
    std::vector<std::size_t> collect_overlapping_primitive_indices(const Aabb3<scalar_type>& query_bounds) const;

    /// Returns all triangle hits whose geometry intersects the query triangle.
    std::vector<const TriangleRecord*> collect_intersecting_triangles(
        const Triangle3<scalar_type>& query_triangle) const;

    /// Returns all triangle hits whose geometry intersects the query quadrilateral.
    std::vector<const TriangleRecord*> collect_intersecting_triangles(const Quad3<scalar_type>& query_quad) const;

    /// Returns unique primitive indices whose triangulated geometry intersects the query triangle.
    std::vector<std::size_t> collect_intersecting_primitive_indices(
        const Triangle3<scalar_type>& query_triangle) const;

    /// Returns unique primitive indices whose triangulated geometry intersects the query quadrilateral.
    std::vector<std::size_t> collect_intersecting_primitive_indices(const Quad3<scalar_type>& query_quad) const;

    /// Visits every triangle whose AABB overlaps the query box.
    template <typename Callback>
    void visit_overlapping_triangles(const Aabb3<scalar_type>& query_bounds, Callback&& callback) const;

    /// Visits every triangle whose geometry intersects the query triangle.
    template <typename Callback>
    void visit_intersecting_triangles(const Triangle3<scalar_type>& query_triangle, Callback&& callback) const;

    /// Visits every triangle whose geometry intersects the query quadrilateral.
    template <typename Callback>
    void visit_intersecting_triangles(const Quad3<scalar_type>& query_quad, Callback&& callback) const;

    /// Exposes the triangulated records for debugging and tests.
    const std::vector<TriangleRecord>& triangles() const;

private:
    /// Stores one BVH node over a contiguous triangle range.
    struct Node {
        Aabb3<scalar_type> bounds = Aabb3<scalar_type>::empty();
        std::size_t first_triangle = 0;
        std::size_t triangle_count = 0;
        std::size_t left_child = invalid_node_index;
        std::size_t right_child = invalid_node_index;

        /// Returns true when the node stores triangles directly.
        bool is_leaf() const;
    };

    static constexpr std::size_t invalid_node_index = std::numeric_limits<std::size_t>::max();

    /// Resets the BVH to the empty state.
    void clear();

    /// Builds one triangle record from a normalized triangle.
    TriangleRecord make_triangle_record(const Triangle3<scalar_type>& triangle,
                                        std::size_t primitive_index,
                                        std::size_t local_triangle_index) const;

    /// Returns the combined bounds of one triangle range.
    Aabb3<scalar_type> compute_range_bounds(std::size_t begin, std::size_t end) const;

    /// Returns the bounds of triangle centroids over one range.
    Aabb3<double> compute_centroid_bounds(std::size_t begin, std::size_t end) const;

    /// Chooses the axis with the largest centroid extent.
    int choose_split_axis(const Aabb3<double>& centroid_bounds) const;

    /// Visits every triangle whose AABB overlaps the query box.
    template <typename Callback>
    void visit_overlapping_triangle_records(const Aabb3<scalar_type>& query_bounds, Callback&& callback) const;

    /// Visits every triangle intersecting any query triangle exactly once.
    template <std::size_t QueryTriangleCount, typename Callback>
    void visit_intersecting_triangle_records(
        const std::array<Triangle3<scalar_type>, QueryTriangleCount>& query_triangles,
        Callback&& callback) const;

    /// Builds one node recursively over a triangle subrange.
    std::size_t build_node(std::size_t begin, std::size_t end);

    std::size_t leaf_size_ = 4;
    std::size_t primitive_count_ = 0;
    std::size_t root_node_index_ = invalid_node_index;
    std::vector<TriangleRecord> triangles_;
    std::vector<Node> nodes_;
};

}  // namespace bvh3d
}  // namespace zcode

#include "bvh3d_impl.h"

#endif
