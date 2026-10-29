# 2-RSS-1U Kinematics Solver

一个独立、可复用、面向实时控制的 2-RSS-1U 并联机构运动学求解库。

本仓库只公开 2-RSS-1U 机构层：解析逆运动学、数值正运动学、解析 Jacobian、速度关系、虚功一致的力矩关系、装配支连续选择和奇异性诊断。整腿 12×12 传动矩阵、虚拟关节/假电机映射、电机通信和机器人控制器均不在公开范围内。

完整推导见：

- [排版后的开源 PDF](output/pdf/2RSS1U_运动学完整推导.pdf)

公开仓库只发布最终 PDF，不包含 LaTeX、Markdown、Office、Visio 或 CAD 原始文档。

## 为什么重新整理这个仓库

论文 *On the Comprehensive Kinematics Analysis of a Humanoid Parallel Ankle Mechanism* 系统给出了固定驱动轴布局下的逆运动学、Jacobian 和 Newton-Raphson 正运动学。本项目以该求解链路为基础，将实际控制代码中已经验证的工程处理整理成独立库，并把支链表达扩展为参数化空间轴模型。

旧代码中常见的

```cpp
AB = AP + Ry(pitch) * Rx(roll) * PB;
AC = Ry(theta) * AC0;
```

在本库中对应为：

```text
moving point = platform_offset + R_y(pitch) R_x(roll) platform_point
crank point  = axis_point + R(axis, theta + offset) crank_zero
```

其中 `platform_offset` 是万向副中心 (O) 在当前支链计算坐标系中的固定坐标
（旧代码中的 `AP`），`platform_point` 是平台零位时从 (O) 指向球副
(B_i) 的固连安装向量（旧代码中的 `PB`）。因此，前者不属于动平台固连偏置，
只有后者随平台姿态旋转。若直接采用以 (O) 为原点的公共基座坐标系，
`platform_offset` 应取零。

支链计算坐标系只允许与公共基座坐标系轴向平行、原点发生平移；
`axis_point`、`platform_offset`、端点和所有支链向量必须在同一坐标系表达。
若局部坐标系相对公共基座系还存在旋转，调用求解器前必须先显式完成该旋转变换。

当 `axis = UnitY()`、`axis_point = 0` 时，公式严格退化为论文和最新控制代码使用的固定 y 轴形式。

## 本项目的实际贡献

### 1. 将固定 y 轴公式扩展为任意空间驱动轴

每条 RSS 支链独立给出轴上一点、轴方向、零位曲柄向量、万向副中心固定坐标和平台球铰零位安装向量。两条电机轴可以平行、正交、斜交或采用左右镜像布局；改变机械布局时只需替换参数。

### 2. 解析逆解返回两个装配根并连续选支

闭环方程统一化为

\[
K_i\cos\phi_i+M_i\sin\phi_i+N_i=0,
\]

程序计算两个解析根，并以当前/上一周期曲柄角为参考选择最近根，避免连续轨迹中因 `atan` 或单根公式产生跳支。

### 3. 补齐可用于控制的异常状态

不可达、三角方程退化、电机限位、支链分母奇异、约束 Jacobian 奇异和正解迭代超限均返回明确状态；不使用零向量冒充有效解，也不在实时函数中抛异常。

### 4. 解析 2×2 Jacobian 与热启动正解

直接构造

\[
\dot{\boldsymbol\theta}=\mathbf J\dot{\mathbf q},
\]

避免 `2×6`、`6×2` 临时矩阵。正运动学沿用论文的

\[
\mathbf q_{k+1}=\mathbf q_k-
\mathbf J^{-1}\left(f_{IK}(\mathbf q_k)-\boldsymbol\theta_m\right),
\]

并加入上一帧热启动、最大迭代次数、最大单步角度和显式奇异检测。

### 5. 速度和力矩使用同一个坐标定义

速度由 `theta_dot = J * q_dot` 给出；力矩严格按虚功得到

\[
\boldsymbol\tau_q=\mathbf J^T\boldsymbol\tau_\theta.
\]

测试同时检查解析 Jacobian 与中心差分的一致性以及两侧瞬时功率的一致性。

## 创新边界

以下内容是公开的基础数学或已有方法，本项目不把它们单独宣称为原创：

- Rodrigues 旋转公式；
- 正弦/余弦三角方程求根；
- Newton-Raphson 迭代；
- Jacobian、奇异性和虚功原理；
- 上述论文所给出的固定布局 2-RSS-1U 求解链路。

本项目希望清楚表达的贡献是：**空间驱动轴参数化、双根连续选择、与最新实时代码一致的完整 IK/FK/Jacobian 实现、异常诊断以及可复现测试**。

“能描述任意空间驱动轴”不等于“无需验证即可控制任意机构”。使用者仍需满足 2-RSS-1U 拓扑与刚体假设，并正确标定轴方向、零位、球铰中心、杆长、角度顺序和装配支。

## 项目结构

```text
include/rss1u/solver.hpp       公共 API 和数据结构
src/solver.cpp                 IK、FK、Jacobian 与速度/力矩关系
examples/example_geometry.hpp  论文兼容参数和任意空间轴示例
examples/basic_demo.cpp        最小调用示例
configs/validation-I.json      构型 I 固定 x 轴复现参数（version 2）
configs/validation-II.json     构型 II 固定 y 轴复现参数（version 2）
configs/validation-III.json    构型 III 非平行空间轴复现参数（version 2）
apps/visualizer/               离线交互式三维运动学可视化网页
apps/studio/                   Qt 6/QML/Quick 3D 桌面工作台
tests/test_solver.cpp          C++ 回归测试
tests/test_solver_failures.cpp 异常与退化状态回归测试
tests/test_interactive_state.cpp  四变量自动 IK/FK 状态回归测试
tests/test_visualizer.js       网页模型与表达式回归测试
tests/reference_validation.py  独立 NumPy 公式验证
tests/batch_validation.py      三构型各 1000 姿态的批量统计验证
tests/README.md                测试矩阵与统一运行说明
output/pdf/                    唯一公开的排版后技术 PDF
```

## 依赖与编译

- C++17
- CMake 3.16+
- Eigen 3.3+

Ubuntu/Debian：

```bash
sudo apt-get install cmake ninja-build libeigen3-dev
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
./build/rss1u_demo
```

没有 C++ 工具链时，可以先运行独立参考验证：

```bash
python tests/reference_validation.py
python tests/batch_validation.py --config-dir configs
```

三份公开 JSON 与网页“添加/管理构型”的导入格式相同。重新生成验证曲线可执行：

```bash
python tests/batch_validation.py --config-dir configs --figures build/validation
```

## 交互式可视化

直接双击打开 [apps/visualizer/index.html](apps/visualizer/index.html)，无需安装、
无需联网，也不需要启动服务器。页面提供：

- 鼠标拖动旋转三维视角、滚轮缩放、双击复位；
- Pitch/Roll 连续调节与轨迹动画；
- 两条电机输出轴方位角、仰角的独立调节；
- 构型 I 固定 x 轴、构型 II 固定 y 轴与任意空间轴三套预设；
- “添加/管理构型”提供中文参数说明，可将自定义构型保存在当前浏览器，并支持 JSON 导入、导出和删除；
- 构型参数支持 `sin`、`cos`、`tan`、`sqrt`、`pi/π` 和角度符号 `°` 等公式输入，并在保存与 JSON 中保留原始表达式；
- 解析 IK 双根连续选择、闭环长度误差、K/M/N、Jacobian 行列式与条件数实时显示；
- 不可达、限位和奇异状态的显式提示。

浏览器端公式逐项对应 `src/solver.cpp`。如果本机安装了 Node.js，可运行
下列回归检查，验证两套几何参数的 IK/FK 回环和解析 Jacobian：

```bash
node tests/test_visualizer.js
```

## Qt 6 桌面工作台

`apps/studio` 提供 **RSS1U Kinematics Studio**。四个变量 Pitch、Roll、θ₁、θ₂
均同时具有滑块和数值输入框，无需手动选择 IK/FK：

- 编辑 Pitch/Roll 自动调用 `rss1u::Solver::inverse()`；
- 编辑 θ₁/θ₂ 自动调用 `rss1u::Solver::forward()`；
- 所有成功结果同步更新另外两个变量、三维端点、闭环残差和 Jacobian；
- 不可达、超限或不收敛时保留最后有效状态；
- QML 只负责显示，分支连续、请求合并和状态提交均由 C++ 管理。

完整构建说明见 [Studio README](apps/studio/README.md)。安装 Qt 6.5+ 后：

```bash
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH=/path/to/Qt/6.x/gcc_64
cmake --build build --target rss1u_studio
./build/rss1u_studio
```

GitHub Actions 会自动编译库和示例，并运行 C++、独立 NumPy、
三构型 1000 姿态批量验证与网页模型测试。

## 最小调用

```cpp
#include <rss1u/solver.hpp>

rss1u::Geometry geometry;
geometry.branches[0] = /* CAD/calibration data */;
geometry.branches[1] = /* CAD/calibration data */;

rss1u::Solver solver(geometry);
rss1u::Vec2 previous_theta = rss1u::Vec2::Zero();
rss1u::Vec2 desired_q(0.08, -0.05);  // [pitch, roll], rad

auto ik = solver.inverse(desired_q, previous_theta);
if (!ik.ok()) {
    // Stop command generation and handle ik.status.
}

auto J = solver.jacobian(desired_q, ik.actuator_angle);
auto fk = solver.forward(ik.actuator_angle, desired_q); // use previous q in practice
```

## 参数与最新代码的对应关系

| 最新控制代码 | 独立库参数 |
|---|---|
| `AP`（支链计算原点到万向副中心 (O)） | `platform_offset` |
| `PB`（(O) 到平台球副 (B_i)） | `platform_point` |
| `AC` | `crank_zero` |
| `Link` | `rod_length` |
| 固定 `Ry(theta)` | `axis = UnitY()`、`axis_point = 0` |
| `q_init` | `forward(..., initial_guess)` |
| `theta_L/theta_R` | `actuator_reference` 或实测执行器角 |

内部统一使用 `[pitch, roll]`，单位为弧度；几何长度单位可以是 mm 或 m，但同一个机构必须一致。

## 验证范围

自动测试覆盖：

- 最新代码参数的零位与非零位 IK/FK 回环；
- 构型 I 固定 x 轴与任意空间驱动轴的 IK/FK 回环；
- 两个解析装配根的闭环长度误差；
- 解析 Jacobian 与中心有限差分；
- 速度与力矩正反映射、关节侧与执行器侧虚功一致性；
- 非法几何、非有限输入、不可达、系数退化、限位、奇异与迭代超限；
- 四变量交互状态更新失败后保留最后有效状态；
- 网页默认构型、公式输入、JSON 往返和非法参数拒绝。

所有自动化测试均集中在 [tests](tests/) 目录，完整测试矩阵与运行方法见
[tests/README.md](tests/README.md)。

示例参数只能证明公式和程序自洽。真实机器还应验证工作空间、球铰摆角、碰撞、编码器符号、结构变形、回差以及目标控制周期内的最坏执行时间。

## 参考文献

1. C. Zhou and N. G. Tsagarakis, “On the Comprehensive Kinematics Analysis of a Humanoid Parallel Ankle Mechanism,” *Journal of Mechanisms and Robotics*, 2018, 10(5): 051015, DOI: [10.1115/1.4040886](https://doi.org/10.1115/1.4040886).
2. R. M. Murray, Z. Li, and S. S. Sastry, *A Mathematical Introduction to Robotic Manipulation*, CRC Press, 1994.

引用相关论文是为了说明基础方法来源，不表示论文作者为本仓库代码背书。

## License

[MIT](LICENSE)

## 发布到 GitHub

```bash
git init
git add .
git commit -m "Initial release: standalone 2-RSS-1U kinematics solver"
git branch -M main
git remote add origin https://github.com/<your-account>/2rss1u-kinematics.git
git push -u origin main
```

建议仓库简介：`Standalone analytic IK, Newton FK and Jacobian solver for 2-RSS-1U parallel mechanisms.`

