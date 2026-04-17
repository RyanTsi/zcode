#include <cassert>
#include <cstddef>
#include <variant>
#include <vector>

#include "../bvh3d.h"

namespace {

using Scalar = double;
using Triangle = zcode::bvh3d::Triangle3<Scalar>;
using Quad = zcode::bvh3d::Quad3<Scalar>;
using Aabb = zcode::bvh3d::Aabb3<Scalar>;
using Vec3 = zcode::bvh3d::Vec3<Scalar>;

/// Verifies that the quad split keeps the smallest angle as large as possible.
void test_quad_triangulation_prefers_best_minimum_angle() {
    const Quad quad{Vec3(-1.0, -1.0, 0.0), Vec3(-0.5, -1.0, 0.0), Vec3(-0.5, -0.5, 0.0), Vec3(-1.0, 0.0, 0.0)};
    const auto triangulation = zcode::bvh3d::split_quad_maximizing_minimum_angle(quad);

    assert(triangulation.diagonal[0] == 0U);
    assert(triangulation.diagonal[1] == 2U);
    assert(triangulation.minimum_angle_degrees > 40.0);
}

/// Verifies that tied quad splits choose a deterministic default diagonal.
void test_quad_triangulation_has_deterministic_tie_break() {
    const Quad quad{Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), Vec3(1.0, 1.0, 0.0), Vec3(0.0, 1.0, 0.0)};
    const auto triangulation = zcode::bvh3d::split_quad_maximizing_minimum_angle(quad);

    assert(triangulation.diagonal[0] == 0U);
    assert(triangulation.diagonal[1] == 2U);
}

/// Verifies that an empty BVH remains queryable and returns no hits.
void test_empty_bvh_query_returns_no_hits() {
    std::vector<Triangle> primitives;
    zcode::bvh3d::Bvh3<Triangle> bvh;
    bvh.build(primitives);

    const auto hits = bvh.collect_overlapping_triangles(
        Aabb{Vec3(-1.0, -1.0, -1.0), Vec3(1.0, 1.0, 1.0)});

    assert(bvh.empty());
    assert(bvh.primitive_count() == 0U);
    assert(bvh.triangle_count() == 0U);
    assert(hits.empty());
}

/// Verifies that triangle-only input can be queried through overlapping AABBs.
void test_triangle_bvh_overlap_query() {
    std::vector<Triangle> primitives{
        Triangle{Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0)},
        Triangle{Vec3(4.0, 4.0, 4.0), Vec3(5.0, 4.0, 4.0), Vec3(4.0, 5.0, 4.0)}};

    zcode::bvh3d::Bvh3<Triangle> bvh(1);
    bvh.build(primitives);

    const auto hits = bvh.collect_overlapping_triangles(
        Aabb{Vec3(-0.1, -0.1, -0.1), Vec3(1.1, 1.1, 0.1)});

    assert(bvh.primitive_count() == 2U);
    assert(bvh.triangle_count() == 2U);
    assert(hits.size() == 1U);
    assert(hits.front()->primitive_index == 0U);
    assert(hits.front()->local_triangle_index == 0U);
}

/// Verifies that misses and invalid query boxes do not report false positives.
void test_triangle_bvh_miss_and_invalid_query() {
    std::vector<Triangle> primitives{
        Triangle{Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0)}
    };

    zcode::bvh3d::Bvh3<Triangle> bvh(1);
    bvh.build(primitives);

    const auto miss_hits = bvh.collect_overlapping_triangles(
        Aabb{Vec3(3.0, 3.0, 3.0), Vec3(4.0, 4.0, 4.0)});
    const auto invalid_hits = bvh.collect_overlapping_triangles(Aabb::empty());

    assert(miss_hits.empty());
    assert(invalid_hits.empty());
}

/// Verifies that mixed triangle and quad input can share one BVH instance.
void test_variant_input_supports_triangle_and_quad() {
    using Primitive = std::variant<Triangle, Quad>;
    std::vector<Primitive> primitives;
    primitives.emplace_back(Triangle{Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0)});
    primitives.emplace_back(Quad{Vec3(2.0, 2.0, 0.0), Vec3(4.0, 2.0, 0.0), Vec3(4.0, 4.0, 0.0), Vec3(2.0, 4.0, 0.0)});

    zcode::bvh3d::Bvh3<Primitive> bvh(1);
    bvh.build(primitives);

    const auto primitive_hits = bvh.collect_overlapping_primitive_indices(
        Aabb{Vec3(1.9, 1.9, -0.1), Vec3(4.1, 4.1, 0.1)});
    const auto triangle_hits = bvh.collect_overlapping_triangles(
        Aabb{Vec3(1.9, 1.9, -0.1), Vec3(4.1, 4.1, 0.1)});

    assert(bvh.primitive_count() == 2U);
    assert(bvh.triangle_count() == 3U);
    assert(primitive_hits.size() == 1U);
    assert(primitive_hits.front() == 1U);
    assert(triangle_hits.size() == 2U);
    for (const auto* hit : triangle_hits) {
        assert(hit->primitive_index == 1U);
    }
}

}  // namespace

int main() {
    test_quad_triangulation_prefers_best_minimum_angle();
    test_quad_triangulation_has_deterministic_tie_break();
    test_empty_bvh_query_returns_no_hits();
    test_triangle_bvh_overlap_query();
    test_triangle_bvh_miss_and_invalid_query();
    test_variant_input_supports_triangle_and_quad();
    return 0;
}
