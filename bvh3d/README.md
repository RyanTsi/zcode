# 3D BVH

该模块提供一个独立的 C++17 三维 BVH，通过模板适配器实现几何无关能力。
对外声明位于 `bvh3d.h`，模板定义位于 `bvh3d_impl.h`。
调用方应只包含 `bvh3d.h`。

核心思路：

- `Bvh3<Primitive, Adapter>` 可以构建在任何“可转换为三角形”的 `Primitive` 类型之上。
- `PrimitiveAdapter` 是实现几何无关扩展的入口。
- `Triangle3<T>` 是树内部统一使用的构建 primitive。
- `Quad3<T>` 会通过选择“最小三角角度最大”的对角线进行三角化。
- 当前构建器采用基于质心的中位数划分，以保持第一版实现简单、确定且紧凑。

当前查询能力：

- 基于三角化记录的 AABB 重叠遍历
- 基于 `Triangle3<T>` 的精确三角形相交查询
- 基于 `Quad3<T>` 的精确四边形相交查询
- 点到最近三角化面的距离查询，返回最近点、位移向量和来源 primitive
- 唯一 primitive 命中收集

查询分层：

- `collect_overlapping_*` / `visit_overlapping_*` 仍然是 broad phase，只做包围盒重叠判断。
- `collect_intersecting_*` / `visit_intersecting_*` 会先用查询图元的 AABB 做 BVH 剪枝，再对候选三角形执行精确几何相交测试。
- 四边形查询会沿用库内的对角线选择策略，先拆成两个三角形，再合并命中结果。
- `find_nearest_face(point)` 会返回距离点最近的三角化面；位移向量方向为 `closest_point - point`。

本轮暂不包含：

- 射线遍历
- 针对非边界顺序、自交或退化四边形的专门处理

文件说明：

- `bvh3d.h`：公开声明
- `bvh3d_impl.h`：由 `bvh3d.h` 引入的模板定义
- `tests/bvh3d_test.cpp`：窄范围编译运行测试

测试命令：

```bash
g++ -std=c++17 -Wall -Wextra -pedantic -I. bvh3d/tests/bvh3d_test.cpp -o /tmp/bvh3d_test
/tmp/bvh3d_test
```

最小示例：

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

Quad query{{1.5, 1.5, 0.0}, {3.5, 1.5, 0.0}, {3.5, 3.5, 0.0}, {1.5, 3.5, 0.0}};
std::vector<std::size_t> hits = bvh.collect_intersecting_primitive_indices(query);

auto nearest = bvh.find_nearest_face({3.0, 3.0, 1.0});
if (nearest) {
    Vec3 displacement = nearest->displacement;
    std::size_t primitive_index = nearest->triangle_record->primitive_index;
}
```
