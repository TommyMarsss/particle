package particle

import "math"

// DegradationWorld 构造“机器人远离一小簇路标、随后径直逼近”的场景。
//
// 传感器有最大探测距离 MaxRange：初始时机器人在路簇探测范围之外，观测为空、
// 权重完全均匀（ESS=N）；随着机器人沿轨迹逼近，进入范围的路标逐个增加，
// 距离梯度变陡、约束迅速收紧，权重集中到少数“幸运”粒子上，ESS 骤降——
// 退化由此可定位到具体时间步。全程不重采样，纯粹展示重要性权重的退化过程。
func DegradationWorld() World {
	return World{
		Bounds: Bounds{MinX: -20, MinY: -15, MaxX: 10, MaxY: 10},
		Landmarks: []Point{
			{0, 0}, {4, 1}, {-3, 3}, {2, -4}, {-2, -2},
		},
		Start:      Pose{X: -14, Y: -7, Theta: 0.3},
		Moves:      straightMoves(12, 0, 1.2),
		RangeNoise: 0.3,
		MaxRange:   9,
	}
}

// MultimodalWorld 构造关于 y 轴镜像对称的环境：所有路标都成对出现在 x=±a 处。
//
// 真实机器人位于 (5, y)，沿平行于对称轴的方向（+y）运动。观测为无标签的
// 纯距离多重集（见 SenseAnonymous），而 (5, y) 与 (-5, y) 对任意路标集合
// 预测出的距离多重集完全相同，似然严格相等——正确的滤波后验应当始终双峰。
func MultimodalWorld() World {
	return World{
		Bounds: Bounds{MinX: -12, MinY: -10, MaxX: 12, MaxY: 10},
		// 关于 y 轴成对镜像，但 y 坐标（−5,−2,1,4,6）刻意不对称：
		// 环境只具有 x→−x 这一个镜像对称性，后验恰为两个峰 (±5, y)，
		// 沿走廊方向（y）则是可观测、可跟踪的。
		Landmarks: []Point{
			{3, -5}, {-3, -5},
			{6, -2}, {-6, -2},
			{3, 1}, {-3, 1},
			{6, 4}, {-6, 4},
			{9, 6}, {-9, 6},
		},
		Start:      Pose{X: 5, Y: -6, Theta: math.Pi / 2},
		Moves:      straightMoves(10, 0, 1.1),
		RangeNoise: 0.15,
	}
}

// MultimodalHypotheses 返回多峰场景中两个镜像假设的判定圆心与半径
// （圆心取运动走廊中点，半径覆盖整段轨迹）。
func MultimodalHypotheses() (Point, Point, float64) {
	return Point{5, -0.5}, Point{-5, -0.5}, 6.0
}

func straightMoves(steps int, turn, dist float64) []Move {
	ms := make([]Move, steps)
	for i := range ms {
		ms[i] = Move{Turn: turn, Dist: dist}
	}
	return ms
}
