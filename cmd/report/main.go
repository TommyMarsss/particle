// 命令 report 运行三组粒子滤波实验并生成单一、完全内嵌的静态 report.html
// （数据与原生 JS/CSS 全部内联，不启动服务、不依赖任何第三方库）。
package main

import (
	"encoding/json"
	"fmt"
	"math"
	"math/rand"
	"os"
	"path/filepath"
	"strings"

	"github.com/TommyMarsss/particle"
)

const degThresholdRatio = 0.25 // 退化告警阈值 ESS < 0.25·N

func main() {
	report := buildReport()
	out := filepath.Join(".", "report.html")
	if err := os.WriteFile(out, []byte(report), 0o644); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	fmt.Printf("wrote %s (%d KB)\n", out, len(report)/1024)
}

func buildReport() string {
	degFrames := runDegradation()
	multiFrames := runMultimodal()
	cmp := runComparison()

	data := map[string]any{
		"deg":   degFrames,
		"multi": multiFrames,
		"cmp":   cmp,
	}
	raw, err := json.Marshal(data)
	if err != nil {
		panic(err)
	}
	return strings.Replace(pageHTML, "__DATA__", string(raw), 1)
}

// ---------- 实验一：退化检测（纯重要性采样，不重采样） ----------

func runDegradation() map[string]any {
	const n = 2000
	rng := rand.New(rand.NewSource(20260930))
	w := particle.DegradationWorld()
	frames := particle.RunDegradation(rng, n, w,
		particle.MotionNoise{SigmaTurn: 0.04, SigmaDist: 0.08}, 0.4)
	threshold := degThresholdRatio * n
	start := particle.DegradationStart(frames, threshold)

	return map[string]any{
		"title":     "实验一 · 粒子退化检测（纯重要性采样，全程不重采样）",
		"n":         n,
		"bounds":    boundsList(w.Bounds),
		"landmarks": pointsList(w.Landmarks),
		"maxRange":  w.MaxRange,
		"threshold": threshold,
		"degStep":   start,
		"essSeries": essSeries(frames),
		"frames":    exportFrames(frames, n, true),
	}
}

// ---------- 实验三：对称环境多峰维持 ----------

func runMultimodal() map[string]any {
	const n = 30000
	rng := rand.New(rand.NewSource(31415))
	w := particle.MultimodalWorld()
	w.MaxRange = 5
	h1, h2, radius := particle.MultimodalHypotheses()
	frames := particle.RunMultimodal(rng, n, w,
		particle.MotionNoise{SigmaTurn: 0.03, SigmaDist: 0.05}, 1.2,
		0.5, 0.4, 0.25, 0.05, h1, h2, radius)

	ess := essSeries(frames)
	m1 := make([]float64, len(frames))
	m2 := make([]float64, len(frames))
	bal := make([]float64, len(frames))
	for i, f := range frames {
		m1[i] = round(f.Mode1Share, 3)
		m2[i] = round(f.Mode2Share, 3)
		bal[i] = round(f.Balance, 3)
	}
	return map[string]any{
		"title":     "实验三 · 镜像对称环境中的多峰维持（自适应低方差重采样 + 抖动）",
		"n":         n,
		"bounds":    boundsList(w.Bounds),
		"landmarks": pointsList(w.Landmarks),
		"h1":        []float64{h1.X, h1.Y},
		"h2":        []float64{h2.X, h2.Y},
		"radius":    radius,
		"essSeries": ess,
		"m1":        m1,
		"m2":        m2,
		"balance":   bal,
		"frames":    exportFrames(frames, 6000, false),
	}
}

// ---------- 实验二：重采样统计对比（重复试验） ----------

func runComparison() map[string]any {
	const (
		n      = 250
		k      = 60
		trials = 4000
	)
	w := skewedWeights(k)
	multi, low := particle.CompareResampling(w, n, trials, 777)
	theoryVar := particle.MultinomialCountVariance(w[0], n)
	theoryEst := particle.MultinomialEstimatorVariance(w, n)

	return map[string]any{
		"title":        "实验二 · 多项式重采样 vs 低方差重采样（4000 次独立重复试验的蒙特卡洛统计）",
		"trials":       trials,
		"n":            n,
		"k":            k,
		"w0":           w[0],
		"expectMean":   float64(n) * w[0],
		"theoryVar":    theoryVar,
		"theoryEstVar": theoryEst,
		"multi":        exportCmp(multi),
		"low":          exportCmp(low),
	}
}

func exportCmp(st particle.ResamplingStats) map[string]any {
	return map[string]any{
		"mean":         round(st.DominantMean, 2),
		"var":          round(st.DominantVar, 3),
		"estVar":       st.EstimatorVar,
		"countVarMean": round(st.CountVarMean, 4),
		"hist":         st.Histogram,
	}
}

func skewedWeights(k int) []float64 {
	w := make([]float64, k)
	w[0] = 0.37
	tail := 0.0
	for i := 1; i < k; i++ {
		w[i] = math.Pow(0.92, float64(i))
		tail += w[i]
	}
	rem := 1 - w[0]
	for i := 1; i < k; i++ {
		w[i] *= rem / tail
	}
	return w
}

// ---------- 帧导出（量化 + 限量，控制 HTML 体积） ----------

func exportFrames(frames []particle.Frame, cap int, withWeight bool) []map[string]any {
	out := make([]map[string]any, len(frames))
	stride := 1
	if cap > 0 && len(frames[0].Particles) > cap {
		stride = len(frames[0].Particles) / cap
	}
	for i, f := range frames {
		ps := make([][]float64, 0, len(f.Particles)/stride+1)
		for j, p := range f.Particles {
			if j%stride != 0 {
				continue
			}
			if withWeight {
				ps = append(ps, []float64{round(p.X, 2), round(p.Y, 2), round(p.Weight, 5)})
			} else {
				ps = append(ps, []float64{round(p.X, 2), round(p.Y, 2)})
			}
		}
		out[i] = map[string]any{
			"truth": []float64{round(f.Truth.X, 2), round(f.Truth.Y, 2), round(f.Truth.Theta, 3)},
			"ess":   round(f.ESS, 1),
			"rs":    f.Resampled,
			"p":     ps,
		}
	}
	return out
}

func essSeries(frames []particle.Frame) []float64 {
	s := make([]float64, len(frames))
	for i, f := range frames {
		s[i] = round(f.ESS, 1)
	}
	return s
}

func pointsList(ps []particle.Point) [][]float64 {
	out := make([][]float64, len(ps))
	for i, p := range ps {
		out[i] = []float64{p.X, p.Y}
	}
	return out
}

func boundsList(b particle.Bounds) []float64 {
	return []float64{b.MinX, b.MinY, b.MaxX, b.MaxY}
}

func round(x float64, d int) float64 {
	m := math.Pow(10, float64(d))
	return math.Round(x*m) / m
}
