# particle — 基于粒子滤波（蒙特卡洛定位）的机器人定位

纯 Go 标准库实现的二维机器人蒙特卡洛定位（MCL），包含：

- 粒子滤波主循环（运动预测 → 观测加权 → 归一化 → 按 ESS 触发重采样）
- **粒子退化检测**：基于有效粒子数（ESS）时间序列，定位退化开始的具体时间步
- **两种重采样算法**（简单多项式 / 低方差）及其实证统计方差对比
- **多峰置信分布**维护验证（对称环境下的双位姿假设）
- 单文件静态 HTML 报告：逐帧回放粒子分布演化 + ESS 曲线（原生 HTML/CSS/JS，无任何第三方库）

## 运行

```bash
go run ./cmd/pfdemo -out report.html   # 生成报告（-seed 可换随机种子）
open report.html                       # 浏览器打开，逐帧回放
go test ./...                          # 运行全部自动化测试
```

报告内嵌三个场景，可在页面顶部切换：

| 场景 | 说明 |
|---|---|
| 正常跟踪 | 滤波器模型与真实模型一致，ESS 保持健康，不应检出退化 |
| 粒子退化 | 观测模型过度自信（σ=0.03 vs 真实 0.3），权重迅速塌缩，检出退化及其起始步 |
| 多峰分布 | 对称环境（两个地标都在 y 轴上），(x,y) 与 (−x,y) 观测完全等价，粒子集应长期维持两个峰 |

## 粒子滤波模型

**状态**：位姿 `(x, y, θ)`。**控制输入**：速度指令 `(v, ω, Δt)`。

**运动模型**：采用《Probabilistic Robotics》的采样速度模型
（`sample_motion_model_velocity`）——对 `v`、`ω` 及末端旋转分别叠加与
速度量级成正比的高斯噪声（6 个 α 参数），沿圆弧积分得到后继位姿；
`ω→0` 时退化为直线模型。见 `models.go` 的 `MotionModel.Sample`。

**观测模型**：对已知地标的**纯距离**观测，高斯噪声。似然为各地标
高斯核的乘积，以对数似然累加后用 log-sum-exp 技巧归一化权重，避免
数值下溢。见 `SensorModel.LogLikelihood` 与 `Filter.Step`。

**滤波循环**（`filter.go`）：

1. 预测：每个粒子从运动模型采样；
2. 加权：粒子权重 ∝ 观测似然；
3. 归一化并计算 ESS；
4. 当 `ESS < 0.5·N` 时触发重采样（默认低方差重采样），权重重置为均匀。

## 粒子退化检测

**有效粒子数** `ESS = 1 / Σ wᵢ²`，取值从 1（一个粒子独占全部权重，
完全退化）到 N（权重均匀，完全健康）。

**判定准则**（`ess.go`）：当 ESS **连续** `DefaultSustainedSteps`（=3）
个时间步低于 `0.3·N` 时判定为退化，退化起始步为该连续段的第一步。

之所以要求"连续多步"而非单次跌破：重采样前 ESS 单步下坠是滤波器的
正常工作方式（重采样随即恢复权重多样性），只有**持续性**塌缩——重采样
也无法恢复权重多样性——才意味着少数粒子永久主导权重、估计不可信。
三个场景的 ESS 序列验证了该准则的区分能力：正常/多峰场景只有孤立的
单步跌落（不报警），退化场景从第 1 步起持续低于阈值（报警并定位 t=1）。

## 重采样算法对比

`resample.go` 实现两种算法，接口均为 `ResampleFunc`（权重 → 祖先索引）：

- **简单多项式重采样**（`MultinomialResample`）：按权重构造 CDF，独立
  均匀采样 N 次。无偏，但索引 i 被选次数服从二项分布，方差
  `N·wᵢ(1−wᵢ)`，是常用方案中最高的。
- **低方差（系统）重采样**（`LowVarianceResample`）：只抽一个随机偏移
  `r ∈ [0, 1/N)`，在等距点 `r, r+1/N, …` 处采样 CDF。同样无偏，但索引
  i 被选次数只能是 `⌊N·wᵢ⌋` 或 `⌈N·wᵢ⌉`，方差 ≤ 1/4；当 `N·wᵢ` 为整数
  时方差恰为 0。

**选择依据**：低方差重采样在保持无偏的前提下显著降低引入的额外随机性
（即更少的"重采样噪声"），且 O(N) 单次扫描、实现简单，因此作为滤波器
默认方案。多项式重采样保留用于对比。

**实证验证**（`resample_test.go`，非理论断言）：

- `TestResamplersUnbiased`：4000 次重复试验，两种算法的平均选择次数
  均等于 `N·wᵢ`（无偏性）；
- `TestLowVarianceResamplingHasLowerVariance`：200 粒子、偏斜权重、
  2000 次试验，实测总选择方差 多项式 ≈ 190.2 vs 低方差 ≈ 11.7，
  低方差仅为前者的 **~6%**（理论：前者 `N(1−Σwᵢ²)`，后者 ≤ N/4）；
- `TestLowVarianceZeroVarianceOnIntegerCounts`：`N·wᵢ` 全为整数时，
  低方差方案实测方差恰为 0，多项式方案实测 ≈ 理论值 0.75。

## 多峰分布维护

`MultimodalScenario`：两个地标置于 `(0, ±8)`，机器人从 `(6, 1)` 出发
沿 x 轴直行。由于观测只依赖 `x²`，位姿 `(x,y,θ)` 与其镜像 `(−x,y,π−θ)`
对**所有**观测等似然——后验天然双峰。粒子初始化为两个各含一半粒子的
簇。测试 `TestMultimodalScenarioKeepsBothModes` 断言：整个运行后半段
每一帧两个峰的权重质量都 ≥ 15%（实测末帧 55.3% / 44.7%），即粒子集
不会过早塌缩到单一（可能是错误的）峰。

## 测试覆盖

```
go test ./... -v
```

| 测试 | 覆盖点 |
|---|---|
| `TestEffectiveSampleSize*` | ESS 计算正确性（均匀→N、单峰→1、已知值） |
| `TestDetectDegeneracy*` | 退化检出、起始步定位、瞬时跌落不误报 |
| `TestResamplersUnbiased` | 两种重采样算法无偏性 |
| `TestLowVarianceResamplingHasLowerVariance` | 低方差 < 多项式方差（实证） |
| `TestLowVarianceZeroVarianceOnIntegerCounts` | 整数计数时低方差方差为 0 |
| `TestDegeneracyScenarioDetected` | 退化场景端到端检出 + 起始步校验 |
| `TestNormalScenarioNotDegenerate` | 正常场景不误报 |
| `TestMultimodalScenarioKeepsBothModes` | 多峰不过早塌缩 |
| `TestNormalScenarioTracksTruth` | 加权均值位姿跟踪真值 |
| `TestMotionModel*` | 运动模型直线/圆弧解析验证 |

## 项目结构

```
particle.go    基本类型（Pose/Control/Landmark/Measurement/Particle）
models.go      运动模型（采样速度模型）与观测模型（纯距离高斯）
ess.go         有效粒子数与退化检测
resample.go    多项式 / 低方差重采样 + 选择次数方差估计
filter.go      粒子滤波主循环
scenario.go    三个内置场景（正常 / 退化 / 多峰）
report.go      单文件静态 HTML 报告生成（数据与逻辑全部内嵌）
cmd/pfdemo/    命令行入口：运行场景并输出 report.html
```
