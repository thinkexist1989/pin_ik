# PIN-IK 与 KDL TRAC-IK 对比

`ik_benchmark.cpp` 使用同一份输入生成与计时代码，分别链接优化前 PIN-IK、阶段 A PIN-IK 和 KDL TRAC-IK。三个版本在独立进程中运行，避免其 `NLOPT_IK` 同名符号互相覆盖。

模型包含带连续关节的 3P+3R 链和有限关节的 6R 链。URDF、seed、目标和参数由共同代码确定；每个后端的原生 FK 先与独立的解析变换链交叉验证。输出解再通过解析 FK 检查位置与姿态残差、关节限位。3P+3R 在所测范围内还比较对应关节解；6R 可能有多个有效分支，不要求关节值完全一致。

## 构建

从仓库根目录执行。所有临时路径均可替换；若使用不同 Pinocchio 安装位置，调整 `pinocchio_DIR`。

```bash
mkdir -p /tmp/pin-ik-stage-a/baseline-source /tmp/pin-ik-stage-a/kdl-source
git archive 553fd1f | tar -x -C /tmp/pin-ik-stage-a/baseline-source
git -C /home/think/Documents/GitHub/trac_ik archive 4335714 | tar -x -C /tmp/pin-ik-stage-a/kdl-source

cmake -S /tmp/pin-ik-stage-a/baseline-source -B /tmp/pin-ik-stage-a/baseline-build -DCMAKE_BUILD_TYPE=Release -Dpinocchio_DIR=/usr/local/lib/cmake/pinocchio
cmake --build /tmp/pin-ik-stage-a/baseline-build -j2
cmake -S . -B /tmp/pin-ik-stage-a/release -DCMAKE_BUILD_TYPE=Release -Dpinocchio_DIR=/usr/local/lib/cmake/pinocchio
cmake --build /tmp/pin-ik-stage-a/release -j2
cmake -S /tmp/pin-ik-stage-a/kdl-source -B /tmp/pin-ik-stage-a/kdl-build -DCMAKE_BUILD_TYPE=Release -DTRAC_IK_BUILD_PYTHON=OFF
cmake --build /tmp/pin-ik-stage-a/kdl-build -j2

cmake -S tests/benchmark -B /tmp/pin-ik-stage-a/bench-a -DCMAKE_BUILD_TYPE=Release -DIK_SOURCE_DIR="$PWD" -DIK_BUILD_DIR=/tmp/pin-ik-stage-a/release -Dpinocchio_DIR=/usr/local/lib/cmake/pinocchio
cmake --build /tmp/pin-ik-stage-a/bench-a -j2
cmake -S tests/benchmark -B /tmp/pin-ik-stage-a/bench-baseline -DCMAKE_BUILD_TYPE=Release -DIK_SOURCE_DIR=/tmp/pin-ik-stage-a/baseline-source -DIK_BUILD_DIR=/tmp/pin-ik-stage-a/baseline-build -Dpinocchio_DIR=/usr/local/lib/cmake/pinocchio
cmake --build /tmp/pin-ik-stage-a/bench-baseline -j2
cmake -S tests/benchmark -B /tmp/pin-ik-stage-a/bench-kdl -DCMAKE_BUILD_TYPE=Release -DIK_BACKEND=KDL -DIK_SOURCE_DIR=/tmp/pin-ik-stage-a/kdl-source -DIK_BUILD_DIR=/tmp/pin-ik-stage-a/kdl-build
cmake --build /tmp/pin-ik-stage-a/bench-kdl -j2
```

KDL 提交 `4335714` 是指定本地仓库迁移至 Pinocchio 之前的 standalone C++17 版本，仍使用 KDL Newton 与 NLopt 双求解器。它不等同于该目录当前 HEAD；归档及构建不会修改原仓库。

## 运行与数据解释

```bash
ctest --test-dir /tmp/pin-ik-stage-a/release --output-on-failure
python3 tests/benchmark/compare.py \
  --solver stage_a=/tmp/pin-ik-stage-a/bench-a/ik_benchmark \
  --solver baseline=/tmp/pin-ik-stage-a/bench-baseline/ik_benchmark \
  --solver kdl=/tmp/pin-ik-stage-a/bench-kdl/ik_benchmark \
  --output /tmp/pin-ik-stage-a/results
```

先完成构建和其他负载测试，再运行计时。默认每组预热 100 次，正确性每组 200 次，五种模式的求解预算为 20 ms；性能每组 20,000 次，使用 `Speed`、550 μs 预算。所有后端容差均为 `1e-6`。计时只包围完整的 `CartToJnt()` 调用，包含其阻塞和内部结果整理；生成输入、调用前类型转换、预热、验证及文件输出不计入。均值和最大值包含失败调用，成功率另列。

`compare.py` 逐组保存 CSV 和 `summary.json`，核对三个后端的 seed 与目标逐项相同。它要求宽预算正确性组全部可达目标成功、不可达组零成功、所有成功解通过残差与限位检查。短预算性能组允许超时，必须连同成功率解释耗时。

平滑轨迹的 seed 为已知目标关节值加交替 ±0.01 的固定扰动，三后端输入完全相同；它是可复现的近目标工作负载，不是实际闭环跟随误差模型。`exact` 的 seed 已满足目标，用于观察调度开销。`jump`、`limits`、`singular`、`unreachable` 用于补充正确性场景。

`singular` 仅用于 6R 模型：以第二关节为 `-π/2`、其余关节为零的秩退化位形为中心，施加 0.001 rad 量级扰动。3P+3R 模型的 pitch 限位排除了其姿态奇异点，不将零位误标为奇异位形。

默认额外执行 10,000 次按绝对时刻释放的 1 kHz 调用，记录从预定释放时刻到求解返回的耗时及超期次数；该测试不包含真实控制器的采集、控制律和通信工作。它不能替代方案中后续的百万周期整机验收。

单组运行参数：

```bash
/tmp/pin-ik-stage-a/bench-a/ik_benchmark /tmp/arm6.csv 20000 0.00055 0 smooth arm6 0
```

依次为输出文件、样本数、求解预算秒数、模式编号、场景、模型、释放周期微秒（0 表示紧密调用）。重复运行时使用不同输出目录保留原始数据。

可通过 `compare.py --suite correctness`、`--suite performance`、`--suite periodic` 分别运行三组测试，并为每组指定独立输出目录。性能组应在编译和检测工具退出后单独运行。

## 线程与检测构建

线程生命周期单独跟踪，不将跟踪器下的耗时用于性能结论：

```bash
strace -f -e trace=clone,clone3 -o /tmp/pin-ik-stage-a/threads.log \
  /tmp/pin-ik-stage-a/bench-a/ik_benchmark /tmp/thread-check.csv 100 0.02 0 exact arm6 0
```

预热和正式调用共 200 次，阶段 A 应只创建两个工作线程，并在进程结束前退出；修改前版本应创建 400 个工作线程。

ThreadSanitizer 示例：

```bash
setarch x86_64 -R cmake -S . -B /tmp/pin-ik-stage-a/tsan \
  -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_STANDARD=17 \
  -Dpinocchio_DIR=/usr/local/lib/cmake/pinocchio \
  '-DCMAKE_CXX_FLAGS=-O1 -g1 -fsanitize=thread -fno-omit-frame-pointer' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread' \
  '-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=thread'
cmake --build /tmp/pin-ik-stage-a/tsan --target worker_tests -j2
setarch x86_64 -R /tmp/pin-ik-stage-a/tsan/tests/worker_tests tests/robot.urdf
```

`setarch -R` 仅对该诊断进程关闭地址随机化，用于避免本机 GCC 11 TSan 的地址映射启动错误；不修改系统全局配置，不用于正式性能测试。

地址和未定义行为检测使用以下独立构建：

```bash
cmake -S . -B /tmp/pin-ik-stage-a/asan \
  -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_STANDARD=17 \
  -Dpinocchio_DIR=/usr/local/lib/cmake/pinocchio \
  '-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer -DEIGEN_MALLOC_ALREADY_ALIGNED=1' \
  '-DCMAKE_CXX_FLAGS_DEBUG=-O1 -g1' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined' \
  '-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=address,undefined'
cmake --build /tmp/pin-ik-stage-a/asan --target worker_tests standalone_tests -j2
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir /tmp/pin-ik-stage-a/asan --output-on-failure -R 'persistent_workers|cpp_standalone'
```

本机预编译 Pinocchio 3.9.0 使用 Eigen 的 16 字节对齐 malloc 约定；GCC ASan 默认会改变 Eigen 对 malloc 的判断。这里显式匹配 `EIGEN_MALLOC_ALREADY_ALIGNED=1`，避免跨库分配与释放使用不同约定。该设置只适用于本机依赖及编译配置；更换依赖或启用更宽的向量对齐时，应统一重建依赖并核对分配约定。

## 阶段 B 复现

阶段 B 采用同一份 `ik_benchmark.cpp` 和 `compare.py`。将当前源码构建到 `/tmp/pin-ik-stage-b/release`；阶段 A 必须使用 `c085321` 归档的头文件和对应库，避免类布局不一致。已有阶段 A 的已验证二进制可直接使用。

```bash
mkdir -p /tmp/pin-ik-stage-b/stage-a-source
git archive c085321 | tar -x -C /tmp/pin-ik-stage-b/stage-a-source
cmake -S /tmp/pin-ik-stage-b/stage-a-source -B /tmp/pin-ik-stage-b/stage-a-build -DCMAKE_BUILD_TYPE=Release -Dpinocchio_DIR=/usr/local/lib/cmake/pinocchio
cmake --build /tmp/pin-ik-stage-b/stage-a-build -j2
cmake -S . -B /tmp/pin-ik-stage-b/release -DCMAKE_BUILD_TYPE=Release -Dpinocchio_DIR=/usr/local/lib/cmake/pinocchio
cmake --build /tmp/pin-ik-stage-b/release -j2
ctest --test-dir /tmp/pin-ik-stage-b/release -V

cmake -S tests/benchmark -B /tmp/pin-ik-stage-b/bench-b -DCMAKE_BUILD_TYPE=Release -DIK_SOURCE_DIR="$PWD" -DIK_BUILD_DIR=/tmp/pin-ik-stage-b/release -Dpinocchio_DIR=/usr/local/lib/cmake/pinocchio
cmake --build /tmp/pin-ik-stage-b/bench-b -j2
cmake -S tests/benchmark -B /tmp/pin-ik-stage-b/bench-a -DCMAKE_BUILD_TYPE=Release -DIK_SOURCE_DIR=/tmp/pin-ik-stage-b/stage-a-source -DIK_BUILD_DIR=/tmp/pin-ik-stage-b/stage-a-build -Dpinocchio_DIR=/usr/local/lib/cmake/pinocchio
cmake --build /tmp/pin-ik-stage-b/bench-a -j2
python3 tests/benchmark/compare.py --suite correctness \
  --solver stage_b=/tmp/pin-ik-stage-b/bench-b/ik_benchmark \
  --solver stage_a=/tmp/pin-ik-stage-b/bench-a/ik_benchmark \
  --solver kdl=/tmp/pin-ik-stage-a/bench-kdl/ik_benchmark \
  --output /tmp/pin-ik-stage-b/correctness
```

KDL 构建仍按前文进行。将 `--suite` 改为 `performance` 或 `periodic`、使用独立输出目录即可复现耗时测试；重复轮次也保留独立目录。Sanitizer 构建沿用前文配置，目录改为 `stage-b`，同时构建并运行 `solver_tests` / `solver_deadlines`。

Linux/glibc 下的分配诊断：

```bash
LD_PRELOAD=/tmp/pin-ik-stage-b/bench-b/liballocation_counter.so \
  /tmp/pin-ik-stage-b/bench-b/allocation_probe tests/robot.urdf
LD_PRELOAD=/tmp/pin-ik-stage-b/bench-a/liballocation_counter.so \
  /tmp/pin-ik-stage-b/bench-a/allocation_probe tests/robot.urdf
```

输出 backend 0/1/2 分别是 Newton、NLopt、双线程入口；每组预热 100 次后计数 1000 次。仅在调用区间启用全进程 `malloc/calloc/realloc` 计数，因此包含工作线程与第三方依赖的对应分配。它不覆盖自定义分配器、直接 `mmap` 或所有 aligned allocator，不证明全调用零分配。不要用 preload 下的耗时评价性能。

## 阶段 C 分项比较

基线为 `599c04f`。报告证据目录保存 `c1.patch`（独立随机状态）、`c2.patch`（锁外评分）、`c3.patch`（梯度修正），依次应用即可重建三个实现。补丁只包含核心 `src/` 和 `include/` 改动，测试工具使用当前仓库版本。

```bash
mkdir -p /tmp/pin-ik-stage-c/b-source
git archive 599c04f | tar -x -C /tmp/pin-ik-stage-c/b-source
cp -a /tmp/pin-ik-stage-c/b-source /tmp/pin-ik-stage-c/c1-source
patch -d /tmp/pin-ik-stage-c/c1-source -p1 < docs/validation/stage_c/c1.patch
cp -a /tmp/pin-ik-stage-c/c1-source /tmp/pin-ik-stage-c/c2-source
patch -d /tmp/pin-ik-stage-c/c2-source -p1 < docs/validation/stage_c/c2.patch
cp -a /tmp/pin-ik-stage-c/c2-source /tmp/pin-ik-stage-c/c-source
patch -d /tmp/pin-ik-stage-c/c-source -p1 < docs/validation/stage_c/c3.patch
```

各版本按前文配置 Release；同时构建 `pin_ik` 和 `pin_ik_worker_test_support`。例如：

```bash
cmake -S /tmp/pin-ik-stage-c/c1-source -B /tmp/pin-ik-stage-c/c1-build -DCMAKE_BUILD_TYPE=Release -Dpinocchio_DIR=/usr/local/lib/cmake/pinocchio
cmake --build /tmp/pin-ik-stage-c/c1-build --target pin_ik pin_ik_worker_test_support -j2
cmake -S tests/benchmark -B /tmp/pin-ik-stage-c/bench-c1 -DCMAKE_BUILD_TYPE=Release -DIK_SOURCE_DIR=/tmp/pin-ik-stage-c/c1-source -DIK_BUILD_DIR=/tmp/pin-ik-stage-c/c1-build -Dpinocchio_DIR=/usr/local/lib/cmake/pinocchio
cmake --build /tmp/pin-ik-stage-c/bench-c1 --target ik_benchmark stage_c_probe -j2
```

`ik_benchmark` 链接生产库，正式性能仍由 `compare.py` 执行；它在可用时调用 `setRandomSeed(42)`，旧版本仍使用 `srand(42)`。不同随机算法不承诺逐个随机数相同，目标/seed 输入序列仍逐项一致。

`stage_c_probe` 链接匹配的测试支持库，比较独立 NLopt 和双线程入口，覆盖近目标 seed 与独立随机 seed，以及五种求解模式。目标/seed 来自固定生成器，每次请求前在计时区间外重置求解器种子。该工具检查每个成功输出的 FK/限位，记录候选数量、最优评分以及 NLopt 最后一次 `optimize()` 的目标函数评估次数。此计数仅统计成功请求，不等于跨随机重启的总迭代数；SLSQP 内部迭代数没有直接导出。工具包含测试钩子判断开销，其时间仅用于同条件诊断比较，不与正式计时混用。

```bash
python3 tests/benchmark/stage_c_compare.py --samples 100 --timeout 0.02 \
  --solver b=/tmp/pin-ik-stage-c/bench-b/stage_c_probe \
  --solver c1=/tmp/pin-ik-stage-c/bench-c1/stage_c_probe \
  --solver c2=/tmp/pin-ik-stage-c/bench-c2/stage_c_probe \
  --solver c=/tmp/pin-ik-stage-c/bench-c/stage_c_probe \
  --output /tmp/pin-ik-stage-c/ablation-wide
```

短预算改为 `--timeout 0.00055`，使用独立输出目录并重复实验。正式完整调用对比将四个版本的 `ik_benchmark` 传给 `compare.py --suite performance` / `--suite periodic`。构建及 Sanitizer 检查结束后再运行计时，不混入诊断负载。

增加 `--urdf tests/benchmark/arm6.urdf` 可测同一正式基准的 6R 几何、限位和工具坐标，检验多个有效分支下的搜索质量。探针仅对连续关节添加 `8π`，不会向有限位的 6R 关节注入越界输入。此次 6R 宽预算每组 100 次，短预算每组 500 次、两轮，第二轮倒序执行版本。

`stage_c_compare.py` 同时输出 `paired-summary.json`：只在两个相邻版本均成功的相同输入用例上比较质量和 NLopt 评估次数。`Distance` 分数越小越好，`Manip*` 越大越好；评分差在 `1e-12 + 1e-4*max(abs(a),abs(b))` 内视为等效。成功率仍独立报告，不用交集过滤失败率。该质量容差只用于统计分类，不改变求解器容差。

已保存 CSV 时可增加 `--analyze-only` 重新汇总而不运行求解器，仍需提供原有 `--samples`、`--timeout` 和 `--solver` 标签。要按 B → C1 → C2 → C 比较，即使实际运行顺序相反，也按这个顺序提供标签。

`gradient_tests` 默认要求数值梯度通过；`--diagnostic` 仅输出误差供历史基线比较。`tests/benchmark/probes` 可将同一梯度测试编译链接到某个历史版本的测试支持库，通过 `IK_SOURCE_DIR`、`IK_BUILD_DIR` 指定匹配路径。

Linux 下另有 `timing_probe`，参数和输入与 `ik_benchmark` 相同，但链接测试支持库并启用工作线程钩子和 `CLOCK_THREAD_CPUTIME_ID`：

```bash
cmake --build /tmp/pin-ik-stage-c/bench-c --target timing_probe -j2
/tmp/pin-ik-stage-c/bench-c/timing_probe /tmp/pin-ik-stage-c/timing-c.csv 10000 0.00055 0 smooth arm6 1000
```

额外列 `dispatch_to_start0/1` 包含发布、通知、锁交接和调度等待；`worker_wall0/1`、`worker_cpu0/1` 分别是每线程求解区间的墙钟和线程 CPU 时间；`first_finish_to_complete` 是第一个工作线程结束求解至两个线程都登记完成的间隔；`last_complete_to_sample` 是最后完成至调用端取样的间隔，含返回整理及包装器复制。单位均为 µs。这些区间不全互斥，不能直接相加。每线程只写自己的槽，调用端在完成屏障后读取。插桩耗时只用于定位，不能替代生产库性能结果。

本次实际批次脚本保存在 `docs/validation/stage_c/run_*.py`；它们假定前述各版本构建已存在。后台负载脚本用 `finally` 终止并回收六个进程，诊断和正式测量分别运行。

## 阶段 D1 线程配置验证

目标控制器尚未在本次会话中测试。以下工具用于在目标机器做同条件配置对比，不会自动选择生产 CPU 或优先级；内置模型仍是 `cartesian`、`arm6` 两条测试链。实际机器人还需使用真实模型、轨迹及控制工作负载验证。

构建新版本库及可选工具（历史版本不提供这些配置 API，不能混用头文件和库）：

```bash
cmake -S . -B build/stage-d -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
cmake --build build/stage-d -j2
ctest --test-dir build/stage-d --output-on-failure
cmake -S tests/benchmark -B build/stage-d-bench -DCMAKE_BUILD_TYPE=Release \
  -DIK_SOURCE_DIR="$PWD" -DIK_BUILD_DIR="$PWD/build/stage-d" \
  -DIK_ENABLE_THREAD_CONFIG_BENCHMARKS=ON
cmake --build build/stage-d-bench --target stage_d_benchmark stage_d_timing_probe -j2
```

按安装位置需要时追加 `-Dpinocchio_DIR=...`。线程配置文件严格包含三个角色，每行四列；`#` 后为注释。仓库自带 `threads-default.conf` 可直接作为基线：

```text
# role   CPUs   policy   priority
caller   -      keep     -
newton   -      keep     -
nlopt    -      keep     -
```

`CPUs` 为逗号分隔整数列表，`-` 表示保持；策略支持 `keep`、`other`、`fifo`、`rr`。`keep` 的优先级列必须为 `-`；`other` 为 `0`，实时策略使用系统允许范围。运行实验前按目标机器资源规划修改文件，不能照抄阶段 D 本机验证中使用的 CPU 编号和优先级。

先执行原设置基线（输出目录必须不存在）：

```bash
python3 tests/benchmark/stage_d_compare.py \
  --executable build/stage-d-bench/stage_d_benchmark \
  --config default=tests/benchmark/threads-default.conf \
  --output /tmp/ik-d-default --samples 10000 --timeout 0.00055 --rounds 2
```

追加 `--config affinity=/path/to/affinity.conf`、`--config scheduling=/path/to/scheduling.conf`、`--config combined=/path/to/combined.conf` 可一次比较多组。默认每 1000 µs 释放；`--period-us 0` 测紧密调用。预算、模式、模型及输入场景通过 `--timeout`、`--mode`、`--robot`、`--scenario` 显式指定。第二轮反转配置执行顺序。样本数量可改为 1,000,000，但这仍是独立基准，不是包含所有控制工作的整机验收。

单次执行格式如下；末尾配置文件是相对于既有基准新增的必选参数：

```bash
build/stage-d-bench/stage_d_benchmark /tmp/ik-d.csv 10000 0.00055 0 smooth arm6 1000 tests/benchmark/threads-default.conf
```

工具在准备输入与结果缓存后设置三个线程，再预热 100 次。库仅配置两个工作线程，基准工具另外显式配置自身调用线程。设置后逐项读回，亲和性被系统缩减或策略/优先级不符均报错；不能把部分生效配置冒充指定实验。多个 setter 不构成事务，失败后本次独立进程退出并销毁工作线程；调用线程通过作用域保护恢复，正常路径还核对恢复结果。它不修改系统级实时配额、隔离 CPU、电源设置或其他进程。

输出 CSV 外另有 `.threads.json`，记录三线程的原始值、请求值、实际值以及调用线程恢复状态。`stage_d_compare.py` 保存配置副本、输入哈希和全部成功/失败状态；权限不足时记为 `unavailable`，进程返回 77，批次返回非零，不能算作通过。没有权限时应由控制器管理员按部署政策配置，工具不自行提权。

使用 `stage_d_timing_probe` 替换 executable 可记录与阶段 C 相同的分段指标，输出与正式测量分别保存，避免用插桩耗时替代生产库耗时。失败行为检查：

```bash
python3 tests/benchmark/stage_d_tool_tests.py --executable build/stage-d-bench/stage_d_benchmark
```

CTest 的实时能力测试若被系统拒绝会明确跳过；权限拒绝测试在独立测试进程中限制自身 `RLIMIT_RTPRIO`，若进程具有可绕过该限制的 `CAP_SYS_NICE` 则明确跳过该项。`worker_thread_unsupported` 在 Linux 上模拟不支持平台分支。跳过不能作为该能力已验证的证据。
