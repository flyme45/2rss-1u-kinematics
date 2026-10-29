# 2-RSS-1U 测试目录

本目录集中保存仓库的全部自动化测试。验证边界仅限公开的 2-RSS-1U
机构层，不包含整腿 12×12 传动、虚拟关节/假电机、通信或机器人控制器。

| 文件 | 主要验证内容 |
|---|---|
| `test_solver.cpp` | 解析 IK 闭环、IK/FK 往返、双装配根、解析 Jacobian 有限差分、速度/力矩映射与虚功一致性；同时覆盖论文兼容示例、构型 I 固定 x 轴和任意空间驱动轴 |
| `test_solver_failures.cpp` | 非法几何、非有限输入、不可达、系数退化、限位、Jacobian 奇异、线性求解失败和 FK 迭代超限 |
| `test_interactive_state.cpp` | Pitch、Roll、θ₁、θ₂ 自动 IK/FK 路由，以及失败后保持最后有效状态 |
| `reference_validation.py` | 读取 `configs/validation-*.json` 后进行独立 NumPy 公式复算，避免只验证 C++ 实现自身 |
| `batch_validation.py` | 读取三份公开 JSON，对三种构型各 1000 个姿态进行闭环、IK/FK 与 Jacobian 统计；可选生成论文结果图 |
| `test_visualizer.js` | 网页求解器、默认构型、参数表达式、三份公开 JSON、一致性往返和非法输入 |
| `test_geometries.hpp` | C++ 测试共享的实际构型参数，仅供测试使用 |

## 运行方法

C++ 测试：

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

独立公式验证：

```bash
python tests/reference_validation.py
python tests/batch_validation.py --config-dir configs
```

如需重新生成论文中的实际结果图：

```bash
python tests/batch_validation.py --config-dir configs --figures build/validation
```

网页模型验证（无需安装 npm 依赖）：

```bash
node tests/test_visualizer.js
```

