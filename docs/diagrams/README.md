# PSAHacks Architecture Diagrams

This directory contains TikZ/LaTeX sources that visualize the relationships between the
major planning and operation components in the simulation.

## Available diagrams

| File | Focus |
| --- | --- |
| `simulation_architecture.tex` | Structural view of how `Simulation`, `PlanningEngine`, and `OperationEngine` collaborate with their supporting resources. |
| `update_cycle_sequence.tex` | Sequence diagram that illustrates the method calls during a single `Simulation.update()` tick. |

## Rendering the diagrams

The sources use the [`standalone`](https://ctan.org/pkg/standalone) document class. Compile each
file with a LaTeX engine that supports TikZ (e.g. `pdflatex` or `xelatex`).

```bash
pdflatex simulation_architecture.tex
pdflatex update_cycle_sequence.tex
```

Each command produces a cropped PDF that can be embedded into documentation or presentations.
