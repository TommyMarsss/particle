package main

import (
	"flag"
	"fmt"
	"os"

	"github.com/TommyMarsss/particle"
)

func main() {
	out := flag.String("out", "report.html", "output HTML file")
	seed := flag.Uint64("seed", 42, "random seed")
	flag.Parse()

	runs := []*particle.RunResult{
		particle.NormalScenario(*seed),
		particle.DegeneracyScenario(*seed),
		particle.MultimodalScenario(*seed),
	}

	for _, r := range runs {
		fmt.Printf("%-8s 步数=%d  退化=%v  退化起始步=%d\n",
			r.Name, len(r.Frames)-1, r.Degeneracy.Degenerate, r.Degeneracy.FirstStep)
	}
	if mm := runs[2]; true {
		last := mm.Frames[len(mm.Frames)-1]
		neg, pos := particle.ClusterMassX(last.Particles)
		fmt.Printf("多峰场景末帧权重质量  x<0: %.1f%%  x>=0: %.1f%%\n", 100*neg, 100*pos)
	}

	f, err := os.Create(*out)
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	defer f.Close()
	if err := particle.WriteHTML(f, runs); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	fmt.Println("已生成", *out)
}
