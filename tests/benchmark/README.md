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

`gradient_tests` 默认要求数值梯度通过；`--diagnostic` 仅输出误差供历史基线比较。`tests/benchmark/probes` 可将同一梯度测试编译链接到某个历史版本的测试支持库，通过 `IK_SOURCE_DIR`、`IK_BUILD_DIR` 指定匹配路径。
