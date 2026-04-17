# 3D BVH

This module provides a standalone C++17 3D BVH that is geometry-agnostic through template adapters.
The public declarations live in `bvh3d.h`, and the template definitions live in `bvh3d_impl.h`.
Consumers should include `bvh3d.h` only.

Core ideas:

- `Bvh3<Primitive, Adapter>` builds over any `Primitive` type that can be converted to triangles.
- `PrimitiveAdapter` is the extension point for geometry-independent use.
- `Triangle3<T>` is the normalized build primitive inside the tree.
- `Quad3<T>` is triangulated by choosing the diagonal that maximizes the minimum triangle angle.
- The current builder uses centroid median splitting to keep the first version deterministic and compact.

Current query support:

- AABB overlap traversal over triangulated records
- Unique primitive hit collection

Out of scope for this iteration:

- Ray traversal
- Exact triangle intersection tests
- Nearest-neighbor or distance queries
- Special handling for non-boundary-ordered, self-intersecting, or degenerate quads

Files:

- `bvh3d.h`: public declarations
- `bvh3d_impl.h`: template definitions included by `bvh3d.h`
- `tests/bvh3d_test.cpp`: narrow compile-and-run coverage

Build and run the test:

```bash
g++ -std=c++17 -Wall -Wextra -pedantic -I. bvh3d/tests/bvh3d_test.cpp -o /tmp/bvh3d_test
/tmp/bvh3d_test
```

Minimal example:

```cpp
#include <variant>
#include <vector>

#include "bvh3d/bvh3d.h"

using Triangle = zcode::bvh3d::Triangle3<double>;
using Quad = zcode::bvh3d::Quad3<double>;
using Primitive = std::variant<Triangle, Quad>;

std::vector<Primitive> primitives{
    Triangle{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}},
    Quad{{2.0, 2.0, 0.0}, {4.0, 2.0, 0.0}, {4.0, 4.0, 0.0}, {2.0, 4.0, 0.0}},
};

zcode::bvh3d::Bvh3<Primitive> bvh;
bvh.build(primitives);
```
