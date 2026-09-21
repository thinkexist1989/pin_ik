# PIN-IK (Pinocchio Inverse Kinematics)

独立的 C++17 逆运动学求解器，基于 Pinocchio 3.9.0 运动学库和 NLopt 优化器。

PIN-IK 是从 TRAC-IK 迁移到 Pinocchio 的版本，完全移除了 KDL 依赖，使用更现代的 Pinocchio 库。

**特点**：
- 双线程并行求解（Jacobian 迭代 + NLopt 优化）
- 五种求解模式：Speed、Distance、Manip1、Manip2、Manip3
- 基于 Pinocchio 3.9.0（支持连续关节和流形几何）
- 完全独立，无需 ROS、catkin、ament、MoveIt 或参数服务器
- 提供 C++ 接口

## 仓库结构

```text
include/      核心 C++ 库公共头文件
src/          核心 C++ 库及求解器实现
examples/     C++ 示例程序
tests/        C++ 自动化测试及测试模型
  manual/     按需构建的手动诊断程序
scripts/      维护脚本（含历史迁移脚本）
docs/         技术文档
cmake/        安装包配置
```

目录使用说明见 [测试目录](tests/README.md) 和 [维护脚本](scripts/README.md)。

可通过 `PIN_IK_BUILD_EXAMPLES=OFF` 关闭对应组件。

仅构建核心库：

```bash
cmake -S . -B build-lib -DCMAKE_PREFIX_PATH=/usr/local \
  -DPIN_IK_BUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF
cmake --build build-lib -j2
```

## 依赖

### 必需
- **CMake** ≥ 3.16
- **C++17** 编译器（GCC ≥ 7, Clang ≥ 5）
- **Eigen3** - 线性代数库
- **Pinocchio** 3.9.0 - 刚体动力学库（运动学）
- **NLopt** - 非线性优化库
- **urdfdom** - URDF 解析（独立版本，非 ROS）
- **Boost** - 序列化支持

### 可选
- **pkg-config** - 简化依赖查找

## 安装依赖

### Ubuntu/Debian
```bash
# 基础构建工具
sudo apt-get install build-essential cmake pkg-config

# 核心依赖
sudo apt-get install libeigen3-dev libnlopt-cxx-dev liburdfdom-dev

# Pinocchio 3.9.0（固定版本，避免安装到 4.x）
git clone --recursive --branch v3.9.0 https://github.com/stack-of-tasks/pinocchio
cd pinocchio
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/openrobots \
  -DBUILD_PYTHON_INTERFACE=OFF
make -j$(nproc)
sudo make install

```

## 编译

### 标准构建
```bash
# 配置
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release

# 如果 Pinocchio 安装在 /opt/openrobots
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/openrobots

# 编译
cmake --build build -j$(nproc)

# 运行测试
ctest --test-dir build --output-on-failure
```

### 构建选项
```bash
# 禁用示例
cmake -S . -B build -DPIN_IK_BUILD_EXAMPLES=OFF

```

## 安装

```bash
# 系统安装（需要 sudo）
sudo cmake --install build --prefix /usr/local

# 用户安装
cmake --install build --prefix ~/.local

# 自定义位置
cmake --install build --prefix /path/to/install
```

安装后：
- 头文件：`<prefix>/include/pin_ik/`
- 库文件：`<prefix>/lib/libpin_ik.so`
- CMake 配置：`<prefix>/lib/cmake/pin_ik/`

## 使用

### C++ API

```cpp
#include <pin_ik/pin_ik.hpp>
#include <pinocchio/spatial/se3.hpp>

// 从 URDF 文件创建求解器
std::string urdf_xml = ...; // 读取 URDF 内容
double timeout = 0.005;     // 5ms 超时
double epsilon = 1e-5;      // 位姿精度

PIN_IK::PIN_IK solver("base_link", "tip_link", urdf_xml, timeout, epsilon);

// 或从 Pinocchio 模型创建
pinocchio::Model model;
Eigen::VectorXd lower_limits, upper_limits;
pinocchio::FrameIndex tip_frame_id;
// ... 加载模型 ...

PIN_IK::PIN_IK solver(model, lower_limits, upper_limits, tip_frame_id, 
                        timeout, epsilon, PIN_IK::Speed);

// 求解 IK
Eigen::VectorXd seed(num_joints);     // 初始关节角度
seed.setZero();

pinocchio::SE3 target_pose;           // 目标位姿
target_pose.translation() << 0.5, 0.0, 0.5;
target_pose.rotation().setIdentity();

Eigen::VectorXd solution;
int result = solver.CartToJnt(seed, target_pose, solution);

if (result >= 0) {
    std::cout << "找到解: " << solution.transpose() << std::endl;
} else {
    std::cout << "未找到解" << std::endl;
}

// 获取所有解（如果有多个）
std::vector<Eigen::VectorXd> solutions;
if (solver.getSolutions(solutions)) {
    std::cout << "找到 " << solutions.size() << " 个解" << std::endl;
}
```

### CMake 集成

```cmake
find_package(pin_ik REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app pin_ik::pin_ik)
```

## 示例

```bash
# 运行 C++ 示例（100 次随机测试）
./build/examples/ik_tests tests/robot.urdf base tip 100

# 运行独立测试
./build/tests/standalone_tests tests/robot.urdf
```

## 求解模式

- **Speed**: 最快速度，可能牺牲精度
- **Distance**: 最小化关节空间距离
- **Manip1**: 最大化操作度（条件数）
- **Manip2**: 最小化条件数倒数
- **Manip3**: 最小化操作度变化

## 性能考虑

- **并发**: 每个 `PIN_IK` 实例不可被多个线程同时调用，但可创建多个实例并行使用
- **超时**: 建议 5-10ms，根据机器人复杂度调整
- **初始化**: 使用接近目标的 seed 可提高成功率和速度

## Pinocchio 3.9.0 与连续关节

CMake 要求 `pinocchio 3.9.0 EXACT`，构建目录应重新配置以清除旧版本缓存。

- C++ 的 seed、解和限位均为 `model.nv` 个标量，连续关节使用弧度，可传入多圈角度。
- 连续关节的限位为 `(-inf, +inf)`，求解结果会选择靠近 seed 的等价角度。
- `getModel()` 返回原生 Pinocchio 模型：连续关节仍占两个配置分量 `(cos θ, sin θ)`，所以 `model.nq` 可能大于 `model.nv`。这在 3.9.0 中同样成立。
- 直接调用 Pinocchio FK 时使用 `PIN_IK::toPinocchioConfiguration(model, angles)` 转换。不要把 IK 返回的角度向量直接当作 Pinocchio 配置。
- 支持轴对齐及任意轴的 revolute、continuous、prismatic 关节。URDF 构造函数提取 base 到 tip 的链，位姿以 base 为参考。
- 有限位关节使用 URDF `<limit>`；不解析 `safety_controller` 的软限位。

```cpp
Eigen::VectorXd angles = Eigen::VectorXd::Zero(model.nv);
Eigen::VectorXd q = PIN_IK::toPinocchioConfiguration(model, angles);
pinocchio::forwardKinematics(model, data, q);
```

## 文档

- **CLAUDE.md**: Claude Code 开发指南
- **docs/MIGRATION_STATUS.md**: KDL 到 Pinocchio 迁移状态
- **docs/FINAL_STATUS.md**: 完整项目状态
- **docs/NEXT_STEPS.md**: Pinocchio 3.9.0 适配指南
- **docs/JACOBIAN_DETAILS.md**: 雅可比矩阵和连续关节详解
- **docs/README_PINOCCHIO.md**: Pinocchio 使用指南

## 迁移说明

本项目已从 Orocos KDL 迁移到 Pinocchio 3.9.0：
- ✅ 所有 KDL 依赖已移除
- ✅ 核心库完全可用
- ✅ 编译和链接成功
- 测试覆盖 C++、连续关节及 FK 残差

## 限制

- base 必须是 tip 的祖先；仅提取该路径上的关节
- 不支持 mimic、floating、planar 关节
- 链中必须至少有一个可动关节
- base 必须是 tip 的祖先

## 故障排除

### Pinocchio 未找到
```bash
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

### 链接错误
确保安装了 Pinocchio 的动态库：
```bash
ls /opt/openrobots/lib/libpinocchio*
```

## 许可证

BSD 3-Clause License - 详见 [LICENSE.txt](LICENSE.txt)

## 致谢

- 原始 TRAC-IK: TRACLabs, Inc.
- PIN-IK (Pinocchio 移植): 基于 TRAC-IK 架构
- Pinocchio: LAAS-CNRS 和 INRIA
- NLopt: Steven G. Johnson

## 引用

如果在研究中使用本库，请引用原始 TRAC-IK 论文：

```bibtex
@inproceedings{beeson2015trac,
  title={TRAC-IK: An open-source library for improved solving of generic inverse kinematics},
  author={Beeson, Patrick and Ames, Barrett},
  booktitle={IEEE-RAS International Conference on Humanoid Robots},
  year={2015}
}
```

PIN-IK 是 TRAC-IK 的 Pinocchio 3.9.0 移植版本，保持了原有的双线程求解架构。
