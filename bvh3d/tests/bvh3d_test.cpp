#include <cassert>
#include <cmath>
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

bool nearly_equal(double left, double right, double tolerance = 1e-12) {
    return std::abs(left - right) <= tolerance;
}

bool nearly_equal(const Vec3& left, const Vec3& right, double tolerance = 1e-12) {
    return nearly_equal(left.x, right.x, tolerance) &&
           nearly_equal(left.y, right.y, tolerance) &&
           nearly_equal(left.z, right.z, tolerance);
}

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
    assert(!bvh.find_nearest_face(Vec3(0.0, 0.0, 0.0)).has_value());
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

/// Verifies that exact triangle tests reject AABB false positives.
void test_triangle_bvh_precise_query_filters_aabb_false_positive() {
    std::vector<Triangle> primitives{
        Triangle{Vec3(0.0, 0.0, 0.0), Vec3(2.0, 0.0, 0.0), Vec3(0.0, 2.0, 0.0)}
    };

    const Triangle query_triangle{
        Vec3(1.6, 1.6, 0.0),
        Vec3(2.5, 1.6, 0.0),
        Vec3(1.6, 2.5, 0.0)
    };

    zcode::bvh3d::Bvh3<Triangle> bvh(1);
    bvh.build(primitives);

    const auto overlap_hits = bvh.collect_overlapping_triangles(
        zcode::bvh3d::detail::compute_triangle_bounds(query_triangle));
    const auto precise_hits = bvh.collect_intersecting_triangles(query_triangle);
    const auto primitive_hits = bvh.collect_intersecting_primitive_indices(query_triangle);

    assert(overlap_hits.size() == 1U);
    assert(precise_hits.empty());
    assert(primitive_hits.empty());
}

/// Verifies that quadrilateral queries are triangulated and matched exactly.
void test_quad_query_reports_precise_intersections() {
    std::vector<Triangle> primitives{
        Triangle{Vec3(0.0, 0.0, 0.0), Vec3(1.5, 0.0, 0.0), Vec3(0.0, 1.5, 0.0)},
        Triangle{Vec3(3.0, 3.0, 0.0), Vec3(4.0, 3.0, 0.0), Vec3(3.0, 4.0, 0.0)}
    };

    const Quad query_quad{
        Vec3(0.25, 0.25, 0.0),
        Vec3(1.0, 0.25, 0.0),
        Vec3(1.0, 1.0, 0.0),
        Vec3(0.25, 1.0, 0.0)
    };

    zcode::bvh3d::Bvh3<Triangle> bvh(1);
    bvh.build(primitives);

    const auto precise_hits = bvh.collect_intersecting_triangles(query_quad);
    const auto primitive_hits = bvh.collect_intersecting_primitive_indices(query_quad);

    assert(precise_hits.size() == 1U);
    assert(precise_hits.front()->primitive_index == 0U);
    assert(primitive_hits.size() == 1U);
    assert(primitive_hits.front() == 0U);
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

/// Verifies that nearest-face queries report the projected point and displacement.
void test_nearest_face_projects_to_triangle_interior() {
    std::vector<Triangle> primitives{
        Triangle{Vec3(0.0, 0.0, 0.0), Vec3(2.0, 0.0, 0.0), Vec3(0.0, 2.0, 0.0)},
        Triangle{Vec3(5.0, 5.0, 5.0), Vec3(6.0, 5.0, 5.0), Vec3(5.0, 6.0, 5.0)}
    };

    zcode::bvh3d::Bvh3<Triangle> bvh(1);
    bvh.build(primitives);

    const auto nearest = bvh.find_nearest_face(Vec3(0.25, 0.25, 3.0));

    assert(nearest.has_value());
    assert(nearest->triangle_record != nullptr);
    assert(nearest->triangle_record->primitive_index == 0U);
    assert(nearly_equal(nearest->closest_point, Vec3(0.25, 0.25, 0.0)));
    assert(nearly_equal(nearest->displacement, Vec3(0.0, 0.0, -3.0)));
    assert(nearly_equal(nearest->distance_squared, 9.0));
    assert(nearly_equal(nearest->distance, 3.0));
}

/// Verifies that nearest-face queries clamp to triangle edges when the projection is outside.
void test_nearest_face_clamps_to_triangle_boundary() {
    std::vector<Triangle> primitives{
        Triangle{Vec3(0.0, 0.0, 0.0), Vec3(2.0, 0.0, 0.0), Vec3(0.0, 2.0, 0.0)}
    };

    zcode::bvh3d::Bvh3<Triangle> bvh(1);
    bvh.build(primitives);

    const auto nearest = bvh.find_nearest_face(Vec3(2.0, 2.0, 1.0));

    assert(nearest.has_value());
    assert(nearly_equal(nearest->closest_point, Vec3(1.0, 1.0, 0.0)));
    assert(nearly_equal(nearest->displacement, Vec3(-1.0, -1.0, -1.0)));
    assert(nearly_equal(nearest->distance_squared, 3.0));
}

/// Verifies that quad primitives are still identified by their original primitive index.
void test_nearest_face_reports_quad_source_primitive() {
    using Primitive = std::variant<Triangle, Quad>;
    std::vector<Primitive> primitives;
    primitives.emplace_back(Triangle{Vec3(10.0, 10.0, 0.0), Vec3(11.0, 10.0, 0.0), Vec3(10.0, 11.0, 0.0)});
    primitives.emplace_back(Quad{Vec3(2.0, 2.0, 0.0), Vec3(4.0, 2.0, 0.0), Vec3(4.0, 4.0, 0.0), Vec3(2.0, 4.0, 0.0)});

    zcode::bvh3d::Bvh3<Primitive> bvh(1);
    bvh.build(primitives);

    const auto nearest = bvh.find_nearest_face(Vec3(3.0, 3.0, 2.0));

    assert(nearest.has_value());
    assert(nearest->triangle_record != nullptr);
    assert(nearest->triangle_record->primitive_index == 1U);
    assert(nearest->triangle_record->local_triangle_index < 2U);
    assert(nearly_equal(nearest->closest_point, Vec3(3.0, 3.0, 0.0)));
    assert(nearly_equal(nearest->displacement, Vec3(0.0, 0.0, -2.0)));
}

}  // namespace

int main() {
    test_quad_triangulation_prefers_best_minimum_angle();
    test_quad_triangulation_has_deterministic_tie_break();
    test_empty_bvh_query_returns_no_hits();
    test_triangle_bvh_overlap_query();
    test_triangle_bvh_miss_and_invalid_query();
    test_triangle_bvh_precise_query_filters_aabb_false_positive();
    test_quad_query_reports_precise_intersections();
    test_variant_input_supports_triangle_and_quad();
    test_nearest_face_projects_to_triangle_interior();
    test_nearest_face_clamps_to_triangle_boundary();
    test_nearest_face_reports_quad_source_primitive();
    return 0;
}
