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
- 唯一 primitive 命中收集

本轮暂不包含：

- 射线遍历
- 精确三角形相交测试
- 最近邻或距离查询
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
```
