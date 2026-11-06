# 阶段 D1：工作线程配置接口与验证工具

日期：2026-11-06。基线：`c8592c5`。用户确认范围为显式工作线程配置及测试工具，保持同步双求解器；目标控制器是另一台机器。

本次完成接口、本机功能回归和工具检查，未接入机器人，未修改 `rocos-app`、系统调度配置、CPU 隔离或电源策略。**这不是目标控制器实时验收，也未确定生产 CPU 编号及优先级。**

## 接口及使用约定

`pin_ik.hpp` 公开三个非实时、空闲阶段接口：

```cpp
void setWorkerThreadAffinity(WorkerThread worker, const std::vector<int>& cpus);
void setWorkerThreadScheduling(WorkerThread worker, int policy, int priority);
WorkerThreadSettings getWorkerThreadSettings(WorkerThread worker);
```

`WorkerThread::Newton`、`WorkerThread::Nlopt` 分别标识两个常驻工作线程。读回结构包含实际 CPU 列表、原生调度策略值及优先级。Linux 调用端通过 `<sched.h>` 使用 `SCHED_OTHER`、`SCHED_FIFO`、`SCHED_RR` 等系统常量。

- 配置和查询不与本实例任何调用或析构重叠；`CartToJnt()` 没有新增配置、查询或线程创建操作。
- 每次 setter 只对一个线程的一项属性进行一次系统设置。默认不改线程已有配置，也不修改调用线程或整个进程。
- 无效 worker、空 CPU 集合、负数/超出 `CPU_SETSIZE` 的 CPU ID、无效策略或优先级范围抛出 `std::invalid_argument`。Linux 本实现支持 CPU ID 小于 `CPU_SETSIZE`（通常为 1024）。
- 系统拒绝设置时抛出 `std::system_error`，保留 pthread 返回的错误码；不会误用 `errno`，也不会静默降级到普通调度。
- Linux 可能把请求亲和性与 cpuset/在线 CPU 限制相交；设置成功后必须读回。基准工具要求实际值与显式请求完全一致，否则拒绝测量。
- 多次 setter 不是事务。绑核成功、随后提权失败时，前一步绑核仍生效。建议在非实时初始化中创建临时实例，逐项设置、读回，全部符合要求后再交给伺服循环；失败则终止初始化并销毁该实例。需要在现有实例上恢复时，由调用端保存原配置并显式恢复。
- `setLimits()` 保留两个线程及其配置；销毁后重新创建 `PIN_IK` 会创建新线程，需要重新应用配置。
- 非 Linux 的有效请求抛出 `std::errc::not_supported`；没有引入平台相关字段改变实例内存布局。非 Linux 分支在 Linux 上通过单独构建进行覆盖，未声称完成其他操作系统的实际编译验收。

控制器初始化示意（CPU、优先级、预算由实际配置提供）：

```cpp
auto ik = std::make_unique<PIN_IK::PIN_IK>(model, lower, upper, tip,
                                         ik_budget_seconds, tolerance);
ik->setWorkerThreadAffinity(PIN_IK::WorkerThread::Newton, newton_cpus);
ik->setWorkerThreadScheduling(PIN_IK::WorkerThread::Newton, SCHED_FIFO, newton_priority);
ik->setWorkerThreadAffinity(PIN_IK::WorkerThread::Nlopt, nlopt_cpus);
ik->setWorkerThreadScheduling(PIN_IK::WorkerThread::Nlopt, SCHED_FIFO, nlopt_priority);
const auto newton_actual = ik->getWorkerThreadSettings(PIN_IK::WorkerThread::Newton);
const auto nlopt_actual = ik->getWorkerThreadSettings(PIN_IK::WorkerThread::Nlopt);
// 校验读回值符合控制器配置后，才发布此实例给伺服循环。
```

调用线程优先级/亲和性、内存锁定、预触页及内核隔离由控制器管理。本次库接口不自动继承或推断伺服优先级，不使用固定 CPU 或优先级作为生产默认值。

## 本机检查

| 检查 | 结果及覆盖 |
| --- | --- |
| Release CTest | 10/10 通过，包含原有 100,000 次线程调用回归及新增配置测试 |
| 安装与下游集成 | 安装到临时前缀后，独立项目仅通过 `find_package(pin_ik)` 和 `pin_ik::pin_ik` 构建，编译并执行 README 的线程配置辅助函数，两个工作线程设置及读回均通过；新增公共头文件已安装 |
| TSan 新增配置测试 | 4/4 通过，无报告 |
| ASan / LSan / UBSan 新增配置测试 | 4/4 通过，无报告 |
| 实际 FIFO / RR | 两个工作线程分别设置优先级 1，原生接口及公共查询读回一致 |
| 实际权限拒绝 | 单独测试进程将自身 `RLIMIT_RTPRIO` 降为 0；真实 `EPERM` 被正确报告，已应用亲和性保持不变 |
| 错误参数与系统拒绝 | 无效 worker、CPU、优先级及内核无效 CPU 掩码被拒绝，未改变被测属性 |
| 生命周期/隔离 | 配置后连续求解 1,000 次通过；更新限位后配置和线程 ID 保持；调用线程未被库接口改动；恢复原设置成功 |
| 不支持的平台分支 | 六次设置/查询均返回不支持；单独模拟构建，不替代跨平台验收 |
| 工具失败处理 | 非法配置拒绝；系统缩减 CPU 掩码时拒绝测量；权限不足记录 `unavailable` 且整个批次返回非零，后续基线组仍可运行 |

TSan 沿用 `setarch x86_64 -R`；显式 C++17 避免本机依赖的默认标准探测执行失败影响配置。ASan 仍用 `EIGEN_MALLOC_ALREADY_ALIGNED=1` 匹配预编译 Pinocchio。检测耗时不作为性能结论。

## 工具与短测

- `stage_d_benchmark`：链接生产库，显式配置调用线程及两个工作线程，计时结束后恢复调用线程，再写 CSV 和实际配置 JSON。
- `stage_d_timing_probe`：同输入与配置，链接测试支持库，增加发布、求解、完成和线程 CPU 分段计时。
- `stage_d_compare.py`：多个配置依次运行，第二轮倒序，保存配置副本、实际生效值、输入序列哈希和统计；不覆盖既有输出目录。权限不足和失败单列，不算测量成功。
- `stage_d_tool_tests.py`：验证非法配置、掩码读回不一致及权限不足的批次报告行为。

本机 i7-1260P、实时内核、550 µs 预算、6R 平滑输入、1 kHz，每组 1,000 次、两轮。仅验证工具工作及设置真正生效；四个本机实验配置如下，**不是目标控制器的建议配置**：

| 配置 | 调用 / Newton / NLopt CPU | 调度 |
| --- | --- | --- |
| default | 保持原设置 | 保持原设置 |
| affinity | 0 / 2 / 4 | 保持原设置 |
| fifo | 保持原设置 | 三线程均 FIFO 1 |
| combined | 0 / 2 / 4 | 三线程均 FIFO 1 |

| 配置 | 第一轮成功 / 1000 | 第一轮最大 µs | 第二轮成功 / 1000 | 第二轮最大 µs |
| --- | ---: | ---: | ---: | ---: |
| default | 1000 | 1230.672 | 1000 | 772.888 |
| affinity | 991 | 4547.288 | 998 | 1188.459 |
| fifo | 1000 | 457.375 | 1000 | 328.152 |
| combined | 1000 | 314.347 | 1000 | 362.076 |

所有成功输出通过 FK/限位检查，各配置输入哈希一致，调用线程恢复均经读回验证。“只绑核”在该短测中更差，不能把绑定 CPU 本身当作稳定性保证。线程实际配置、P99/P99.9、超预算和预定释放至返回超期均保存在汇总中；不能用短测的局部零超期替代整机百万周期验收。

## 目标控制器下一步

按 [工具使用说明](../tests/benchmark/README.md#阶段-d1-线程配置验证)构建并运行。部署前还需确认控制器 CPU 拓扑、现有伺服/总线线程优先级与亲和性、IK 预算和允许的资源占用。依次比较基线、仅绑核、仅实时调度、组合配置，保持其他输入与参数一致，并分别进行有负载/无负载测量。

确认控制器集成端后，显式传入 IK 预算、在重建实例时重施配置，并区分预览实例与伺服实例。单求解器、延迟启动及异步架构不在本次范围内。目标机配置与整机零超期验收仍待执行。

证据保存在 [stage_d](validation/stage_d/)；原始 CSV 位于本机 `/tmp/pin-ik-stage-d/`。本次未改动系统级实时限额、CPU 隔离或机器人进程。
