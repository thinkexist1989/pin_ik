# 阶段 D1 本机验证证据

目标控制器位于另一台机器。本目录记录接口和测试工具的本机验证，不证明目标控制器实时性能。

- `release-tests.log`：10/10；包含已有求解器回归及新增配置测试。
- `asan-tests.log`、`tsan-tests.log`：新增配置测试各 4/4；本机没有跳过实时能力或权限拒绝测试。
- `tool-tests.json`：非法配置、被内核缩减的亲和性掩码、权限不足的工具行为检查。
- `smoke-periodic.json`：四组配置各 1000 周期、两轮；包含三线程原始/请求/实际设置及调用线程恢复状态。
- `smoke-timing.json`：1000 周期的分段诊断工具运行摘要；原始 CSV 另核对时间字段有效性。
- `*.conf`：实际使用的本机配置及失败用例。**其中 CPU 和优先级不可作为目标控制器的生产默认配置。**
- `environment.json`：环境、源码和二进制 SHA-256，以及实际链接的 Sanitizer 运行库。

完整 CSV 在本机 `/tmp/pin-ik-stage-d/`，仓库保存汇总和复现工具。命令和 Sanitizer 配置说明见 [基准文档](../../../tests/benchmark/README.md#阶段-d1-线程配置验证)及[验证记录](../../REALTIME_IK_STAGE_D_VALIDATION.md)。
