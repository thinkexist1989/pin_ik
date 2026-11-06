# 阶段 C 提交证据

本目录保存阶段 C 已完成的检查，不包含尚未运行的分项性能验收结果。结论和限制见 [验证记录](../../REALTIME_IK_STAGE_C_VALIDATION.md)。

- `release-tests.log`：Release 6/6。
- `tsan-tests.log`：TSan 4/4，无屏蔽告警。
- `asan-tests.log`：ASan / LSan / UBSan 5/5。
- `correctness-summary.json`：C、B、KDL 宽预算正确性汇总。
- `gradient-b.log`：B 梯度诊断，旧梯度比例为 0.5；C 的梯度结果在 Release 日志内。
- `forward-original-step-failed-test.log`：修正分母但保留旧步长的中间版本失败记录，不代表最终版本。
- `c1.patch`、`c2.patch`、`c3.patch`：从 `599c04f` 起依次应用的核心实现补丁，最终 `include/` 和 `src/` 内容已核对一致。
- `source-sha256.txt`：本次实现与 C++ 测试源文件的 SHA-256。

测试日志保留当时 `/tmp/pin-ik-stage-c` 的运行路径；这些路径并非仓库文件。详细构建和运行方法见 [基准说明](../../../tests/benchmark/README.md)。
