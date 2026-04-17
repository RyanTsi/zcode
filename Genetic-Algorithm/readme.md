# Genetic Algorithm

`Genetic-Algorithm/` 是一份用于学习遗传算法的独立实验代码。
当前实现维护固定大小的种群，在一个 30 维整数向量空间上进行编码、交叉、变异和选择，并把每一代的统计结果写入 `res`，供后续绘图脚本使用。

当前实验内容：

- 染色体由 30 个基因组成，每个基因包含 5 位数值和 1 位符号位。
- 初始种群随机生成，并为个体适应度提供缓存。
- 交叉阶段采用多点交叉。
- 变异阶段按概率对单个染色体位进行翻转。
- 选择阶段使用锦标赛筛选保留下一代个体。
- 每一代都会输出种群平均目标值，便于观察收敛趋势。

目标函数说明：

- `get_val()` 会计算 30 个带符号整数的平方和，因此当前实验等价于让各维度尽量接近 0。
- `get_fitness()` 使用 `MAXV - get_val()` 作为适应度，目标值越小，适应度越高。
- 符号位会参与编码、交叉和变异，但不会改变平方和目标值本身。

文件说明：

- `main.cpp`：GA 主程序，运行后会把 `代数 平均值` 结果写入 `res`
- `draw.py`：读取 `res` 并使用 `matplotlib` 绘制迭代曲线
- `test.cpp`：独立的小型数值实验文件，不参与主流程
- `res`：一次运行后的结果数据文件，重新执行主程序时会被覆盖
- `Figure_1.png`：已有的绘图结果示意图

构建方式：

```bash
cd Genetic-Algorithm
g++ -std=c++20 -Wall -Wextra -pedantic main.cpp -o ga
```

运行主程序：

```bash
cd Genetic-Algorithm
./ga
```

程序会在当前目录生成或覆盖 `res`。由于源码中使用了 `freopen("res", "w", stdout)`，建议在 `Genetic-Algorithm/` 目录内执行，避免结果文件写到其他路径。

绘制曲线：

```bash
cd Genetic-Algorithm
python3 draw.py
```

运行绘图脚本前需要本地已安装 `matplotlib`。脚本会读取 `res` 并弹出交互式图窗。

附加验证：

```bash
cd Genetic-Algorithm
g++ -std=c++20 -Wall -Wextra -pedantic test.cpp -o ga_test
./ga_test
```

`test.cpp` 当前只是一个保留的独立实验文件，可用于确认本地编译环境可正常处理该目录下的额外源码。
