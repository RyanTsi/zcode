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
Aabb3<T> compute_triangle_bounds(const Triangle3<T>& triangle) {
    Aabb3<T> bounds = Aabb3<T>::empty();
    bounds.expand(triangle.a);
    bounds.expand(triangle.b);
    bounds.expand(triangle.c);
    return bounds;
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
template <typename Callback>
void Bvh3<Primitive, Adapter>::visit_overlapping_triangles(const Aabb3<scalar_type>& query_bounds,
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
