# 测试

- `standalone_tests.cpp`：C++ 回归测试。
- `worker_tests.cpp`：常驻线程的任务序号、受控唤醒、启动失败回收、异常恢复、配置更新和 100,000 次混合调用；测试钩子只编译进独立测试库。
- `solver_tests.cpp`：截止时间传递、四种 NLopt 误差模式的取消输出、差分停止、异常传播与恢复、非零 seed 求解，以及串联/分支模型的数值雅可比检查。收尾与运动学计时只作诊断，不设不稳定的耗时通过门槛。
- `gradient_tests.cpp`：四种 NLopt 误差模式共 600 组配置的梯度与中心差分交叉验证，含多圈连续关节与 bounds 分量屏蔽。
- `optimization_tests.cpp`：独立随机序列、固定种子、限位更新、并行实例、受控评分交错，以及所有候选评分与排序的独立验证。
- `worker_thread_tests.cpp`：工作线程配置、OS 读回、无效参数/实际权限拒绝、配置后的求解与生命周期；Linux 另覆盖 FIFO/RR 和模拟不支持平台分支。实时设置被系统拒绝时对应能力测试明确跳过。
- `benchmark/`：相同工作负载下分别测试 PIN-IK 与 KDL TRAC-IK，校验解并记录完整调用的平均、最大和分位耗时，见 [复现步骤](benchmark/README.md)。
- `robot.urdf`：测试及随机 IK 示例共用的机器人模型。
- `manual/`：手动诊断程序，按需构建，见 [使用说明](manual/README.md)。
- `VALIDATION.md`：历史本地验证记录，当前结果应以实际运行 CTest 为准。

在仓库根目录执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

示例测试随 `PIN_IK_BUILD_EXAMPLES` 开关启用。
