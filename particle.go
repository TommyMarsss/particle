// Package particle 实现基于粒子滤波（蒙特卡洛定位, MCL）的二维机器人定位。
//
// 位姿状态为 (X, Y, Theta)，观测为到已知路标的纯距离（range-only）。
// 包内只依赖 Go 标准库（math、math/rand），不使用任何第三方统计/随机数库。
package particle

import "math"

// Particle 是一个带权重的位姿粒子。
type Particle struct {
	X, Y, Theta float64
	// Weight 是最近一次归一化后的线性权重（和为 1）。
	Weight float64
}

// Pose 表示机器人二维位姿。
type Pose struct {
	X, Y, Theta float64
}

// Point 是二维平面上的一个点（路标位置）。
type Point struct {
	X, Y float64
}

// Bounds 是一个轴对齐矩形区域 [MinX, MaxX] × [MinY, MaxY]。
type Bounds struct {
	MinX, MinY, MaxX, MaxY float64
}

// Move 是一条简化的运动指令：先原地旋转 Turn，再平移 Dist。
type Move struct {
	Turn float64
	Dist float64
}

// MotionNoise 是运动模型的加性高斯噪声标准差。
type MotionNoise struct {
	SigmaTurn float64 // 旋转噪声
	SigmaDist float64 // 平移噪声
}

func wrapAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a < -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// weightedPose 用归一化权重计算加权平均位姿。
// 朝向使用圆循环均值（sin/cos 平均）处理 ±π 跳变。
func weightedPose(ps []Particle) Pose {
	var sx, sy, sw, ss, sc float64
	for _, p := range ps {
		sx += p.X * p.Weight
		sy += p.Y * p.Weight
		ss += math.Sin(p.Theta) * p.Weight
		sc += math.Cos(p.Theta) * p.Weight
		sw += p.Weight
	}
	if sw == 0 {
		return Pose{}
	}
	return Pose{X: sx / sw, Y: sy / sw, Theta: wrapAngle(math.Atan2(ss, sc))}
}

func gaussPDF(delta, sigma float64) float64 {
	z := delta / sigma
	return math.Exp(-0.5*z*z) / (sigma * math.Sqrt2 * math.SqrtPi)
}
