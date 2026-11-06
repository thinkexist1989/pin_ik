# 阶段 C 提交证据

本目录保存阶段 C 正确性、分项性能、周期与负载测量，以及超期分段诊断。结论和限制见 [验证记录](../../REALTIME_IK_STAGE_C_VALIDATION.md)。

- `release-tests.log`：Release 6/6。
- `tsan-tests.log`：TSan 4/4，无屏蔽告警。
- `asan-tests.log`：ASan / LSan / UBSan 5/5。
- `correctness-summary.json`：C、B、KDL 宽预算正确性汇总。
- `gradient-b.log`：B 梯度诊断，旧梯度比例为 0.5；C 的梯度结果在 Release 日志内。
- `forward-original-step-failed-test.log`：修正分母但保留旧步长的中间版本失败记录，不代表最终版本。
- `c1.patch`、`c2.patch`、`c3.patch`：从 `599c04f` 起依次应用的核心实现补丁，最终 `include/` 和 `src/` 内容已核对一致。
- `source-sha256.txt`：核心实现及 C++、Python、URDF 和 CMake 测试/基准源文件的 SHA-256。
- `optimization-arm6.log`：在新增 6R 模型上复用优化回归的通过记录。
- `environment.json`：环境、核心实现提交和测量二进制指纹；生产库未随诊断工具的增补而改变。
- `ablation*.json`：3P+3R、6R 宽/短预算分项，带 `paired` 后缀的文件仅比较双方成功的同一用例。
- `performance-*.json`、`periodic-*.json`：生产库两轮连续/周期调用，对照 B/C1/C2/C/A/KDL 或 B/C/A/KDL。
- `load-*.json`、`load.log`：六个后台负载进程下的生产库测量及回收记录。
- `timing-summary.json`、`timing.log`：独立诊断运行的分段/线程 CPU 计时，不混入生产库统计。
- `run_remaining.py`、`run_load.py`、`run_timing.py`：本次机器上实际执行的批次脚本，使用明确的 `/tmp` 构建目录；运行前按基准说明准备对应构建。`remaining.log` 记录批次顺序。

测试日志保留当时 `/tmp/pin-ik-stage-c` 的运行路径；这些路径并非仓库文件。详细构建和运行方法见 [基准说明](../../../tests/benchmark/README.md)。
