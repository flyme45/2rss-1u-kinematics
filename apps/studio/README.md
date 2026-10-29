# RSS1U Kinematics Studio

这是基于 Qt 6、QML 与 Qt Quick 3D 的桌面交互程序。运动学只调用项目现有的
`rss1u::Solver`；QML 不包含 IK、FK 或 Jacobian 公式。

## 本阶段已实现

- Pitch、Roll、θ₁、θ₂ 四组控件始终可见；
- 每个变量同时提供滑块和可精确编辑的小数输入框；
- 操作 Pitch 或 Roll 时自动调用 IK，并保持另一个平台角不变；
- 操作 θ₁ 或 θ₂ 时自动调用 FK，并保持另一个驱动角不变；
- IK 使用上一有效驱动角作为装配分支参考；
- FK 使用上一有效平台姿态作为初值；
- 失败请求采用事务式回滚，保留最后有效角度、端点和三维姿态；
- 电机滑块范围来自 `motor_limit`；平台角范围由当前构型的连续可达区间扫描得到；
- QML 编辑框在回车或失去焦点时提交，编辑期间不被求解结果覆盖；
- 事件循环仅保留最新拖动请求，避免过期请求排队；
- 自动显示本次更新采用 IK 还是 FK；
- Qt Quick 3D 显示 O、Mᵢ、Aᵢ、Bᵢ、两条电机轴、曲柄、RSS 连杆和动平台骨架；
- 内置构型 I（固定 x 轴）、构型 II（固定 y 轴）和非平行空间驱动轴构型；
- 可按方位角和仰角交互调整两条驱动轴，调整时保持曲柄零位端点和径向长度；
- 中文构型编辑器支持公共万向副中心 O、两条支链参数、JSON 导入导出与本地保存；
- 参数框支持 `sin`、`cos`、`tan`、`sqrt`、`abs`、`pi/π` 和四则运算，三角函数普通数字默认按度解释；
- 纯数字输入的上下按钮根据当前小数位数改变最后一位；
- 实时显示闭环残差、Jacobian、K/M/N 系数、行列式、条件数和 FK 迭代次数。

## 状态流

```text
QML 用户事件
    │
    ▼
StudioStateManager（合并最新请求、属性同步、输入来源识别）
    │
    ▼
InteractiveState（候选状态、分支连续、成功提交/失败回滚）
    │
    ▼
rss1u::Solver（真实 IK / FK / Jacobian / 闭环残差）
```

`InteractiveState` 不依赖 Qt，因此可以在没有图形环境的 CI 中单独回归测试。

## 依赖

- C++17
- Eigen 3.3+
- Qt 6.5+：Quick、QuickControls2、Quick3D
- CMake 3.16+

## Windows 构建

在 Qt 6 的命令行环境中执行：

```powershell
cmake -S . -B build -G Ninja `
  -DCMAKE_PREFIX_PATH=C:/Qt/6.8.3/msvc2022_64 `
  -DCMAKE_BUILD_TYPE=Release
cmake --build build --target rss1u_studio
.\build\rss1u_studio.exe
```

请把示例中的 Qt 目录替换为本机实际版本。使用 MinGW Qt 时，应同时使用与该
Qt 套件匹配的 MinGW 编译器。

## 测试

```powershell
cmake --build build --target rss1u_tests rss1u_studio_state_tests
ctest --test-dir build --output-on-failure
```

若 CMake 未找到 Qt，`rss1u_studio` 图形目标会跳过，但求解器和
`rss1u_studio_state_tests` 仍可正常构建测试。

## 当前边界

Studio 聚焦单机交互、构型管理与真实三维闭环，不连接或下发真实电机控制
指令。批量随机验证仍由 `tests/batch_validation.py` 执行；界面中的构型编辑不会
替代标定流程，也不会通过改变杆长或播放虚假动画掩盖不可达状态。
