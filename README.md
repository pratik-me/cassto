# CASSTO: QoS-Oriented Scientific Workflow Scheduling in Cloud Computing

An enhanced, empirically audited extension of the **WorkflowSim 1.0** toolkit implementing and benchmarking **CASSTO** (*Cloud-Aware Seal Search Task Optimization*) alongside comparative evolutionary metaheuristics (**GA**, **HICA**) and classical list heuristics (**HEFT**, **DHEFT**, **Random**) for scientific workflows in heterogeneous cloud computing environments.

---

## 📌 Academic Context & Background

This repository contains the Java implementation, bug fixes, empirical auditing framework, and benchmarking testbeds corresponding to the study:

> **"CASSTO: a bio-inspired metaheuristic for QoS-oriented scientific workflow scheduling in cloud computing"**  
> *Hameed R. et al., The Journal of Supercomputing (2026).*

Scientific workflow scheduling in cloud environments is an NP-hard multi-objective optimization problem. Schedulers must map workflow tasks (Directed Acyclic Graphs, or DAGs) onto heterogeneous Virtual Machines (VMs) while simultaneously optimizing conflicting Quality of Service (QoS) criteria: **Makespan**, **Financial Cost**, and **Resource Utilization**, while strictly obeying task dependency constraints.

---

## 🔬 Empirical Audit: Key Fixes & Architectural Enhancements

During rigorous replication and ablation studies, several critical architectural defects and confounding design flaws in the original paper/codebase were identified and repaired in [`CASSTOPlanningAlgorithm.java`](sources/org/workflowsim/planning/CASSTOPlanningAlgorithm.java) and [`OrderAwarePlanningBase.java`](sources/org/workflowsim/planning/OrderAwarePlanningBase.java):

### 1. Task Pool Partitioning Decoupling (`cassto.pool=none|urgent`)
* **Original Issue**: Brownian moves picked *only* from normal tasks, and Lévy flights picked *only* from urgent tasks. In `schedule` and `diversity` modes, this created an unintended hard partition: normal tasks were trapped in local VM moves forever, and urgent tasks could only ever take global jumps.
* **Our Enhancement**: Added the explicit property `cassto.pool=none|urgent`. In `schedule` and `diversity` modes, `cassto.pool=none` allows all tasks to undergo both Brownian exploitation and Lévy exploration, preserving `urgent` strictly for critical-path urgency mode.

### 2. Brownian Step Size Execution in Urgency Mode
* **Original Issue**: The paper claimed that Brownian step size $bMoves$ adapts dynamically. However, in `urgency` mode, the search called `brownianTaskMove()`, which had a hardcoded 1-task swap. The variable `currentBrownianMoves` was only read in `brownianMove()`, rendering Brownian adaptation completely inactive in urgency mode!
* **Our Enhancement**: Connected `currentBrownianMoves` directly to `brownianTaskMove()`, allowing it to execute $k = 1 \dots currentBrownianMoves$ swaps per move.

### 3. Scale-Invariant Stagnation Tracking
* **Original Issue**: Stagnation was measured using composite fitness under dynamically shifting Min-Max bounds (Paper Eq. 12). Because bounds updated every iteration, fitness values changed scale, causing false resets and failing to increment the stagnation counter when adaptation was disabled.
* **Our Enhancement**: Stagnation is tracked using the scale-invariant, unnormalized makespan:
  ```java
  double bestRaw = best.rawMetrics[0];
  if (bestRaw < prevBestRaw - 1e-6) {
      stagnationCount = 0;
      alphaBurstDone = false;
      brownianBurstDone = false;
      prevBestRaw = bestRaw;
  } else {
      stagnationCount++;
  }
  ```

### 4. Normalized Relative Population Diversity Threshold
* **Original Issue**: Population diversity $d$ was compared directly against a static threshold $\sim 0.15$ or $0.25$. In 100-task workflow DAGs, pairwise topological Hamming distance rarely drops below $0.70$. As a result, $d < 0.25$ was mathematically unreachable, and the diversity-driven phase switch never fired.
* **Our Enhancement**: Normalized diversity to relative population dispersion: $relD = d / initialDiversity$.

### 5. Stabilized Online Adaptation Feedback Law
* **Original Issue**: The adaptation loop evaluated win rates on iteration 1 (causing zero-attempt spikes), lacked minimum-attempt guards, and multiplied $\alpha$ by $1.15$ on *every* stagnant iteration, saturating $\alpha$ to its upper limit within ~12 iterations.
* **Our Enhancement**:
  - Updates strictly synchronize with window boundaries: `(iter + 1) % WINDOW == 0`.
  - Added sample size protection: `windowLevyAttempts >= MIN_ATTEMPTS`.
  - Replaced runaway scaling with equilibrium exponential tuning: $\alpha \leftarrow \alpha \cdot \exp(\eta \cdot (rate - rate_{target}))$.
  - Introduced independent one-shot burst flags (`alphaBurstDone` and `brownianBurstDone`) cleared only on true makespan improvement.

### 6. Simulated Annealing Brownian Cooling Schedule
* **Our Enhancement**: Added support for continuous step decay (`cassto.brownianDecay=true`), linearly annealing the Brownian step size from $8 \to 1$ over the evaluation budget.

### 7. Unified Multi-Objective Metric Normalization
* **Original Issue**: CASSTO refreshed min-max bounds dynamically while GA fell through to an unnormalized reference branch, skewing cross-algorithm comparisons.
* **Our Enhancement**: Standardized unified reference normalization (`norm=ref`, Eq. 4) across both CASSTO and GA, guaranteeing mathematically fair comparisons.

---

## 🚀 Implemented Algorithms

| Algorithm | Type | Core Class | Search Mechanism |
| :--- | :--- | :--- | :--- |
| **CASSTO** | Bio-inspired Metaheuristic | [`CASSTOPlanningAlgorithm.java`](sources/org/workflowsim/planning/CASSTOPlanningAlgorithm.java) | Hybrid Brownian local search ($bMoves \in [1, 10]$) and heavy-tailed Lévy flight jumps ($\alpha \in [0.5, 6.0]$), with optional online feedback adaptation. |
| **GA** | Evolutionary Metaheuristic | [`GAPlanningAlgorithm.java`](sources/org/workflowsim/planning/GAPlanningAlgorithm.java) | Genetic Algorithm with tournament selection ($k=3$), uniform order crossover ($p_c=0.8$), bit/gene mutation ($p_m=0.05$), and elitism. |
| **HICA** | Evolutionary Metaheuristic | [`HICAPlanningAlgorithm.java`](sources/org/workflowsim/planning/HICAPlanningAlgorithm.java) | Hybrid Imperialist Competitive Algorithm with empire assimilation, colony revolution, and imperial competition. |
| **HEFT** | List Heuristic | [`HEFTPlanningAlgorithm.java`](sources/org/workflowsim/planning/HEFTPlanningAlgorithm.java) | Heterogeneous Earliest Finish Time prioritizing tasks via upward rank values ($rank_u$). |
| **DHEFT** | Dynamic Heuristic | [`DHEFTPlanningAlgorithm.java`](sources/org/workflowsim/planning/DHEFTPlanningAlgorithm.java) | Dynamic variation of HEFT adapting to runtime state changes. |
| **Random** | Baseline | [`RandomPlanningAlgorithm.java`](sources/org/workflowsim/planning/RandomPlanningAlgorithm.java) | Uniformly distributes tasks randomly across available VMs. |

---

## ⚙️ Configurable JVM Properties & Execution Flags

All algorithm parameters can be dynamically injected via `-D` flags without re-compiling:

| Property | Default | Options / Range | Description |
| :--- | :---: | :---: | :--- |
| `cassto.mode` | `urgency` | `urgency`, `schedule`, `diversity` | Exploration/exploitation phase control mode. |
| `cassto.pool` | `urgent` | `none`, `urgent` | Task pool partitioning (`none` lets all tasks move in both phases). |
| `cassto.brownianMoves` | `1` | `1`, `4`, `8`, etc. | Number of task swaps executed per Brownian local move. |
| `cassto.alpha` | `2.0` | `0.5` – `6.0` | Lévy flight power-law distribution exponent. |
| `cassto.adaptive` | `false` | `true`, `false` | Enables/disables online RL win-rate feedback adaptation. |
| `cassto.adaptiveInit` | `false` | `true`, `false` | Enables/disables DAG topology-based parameter initialization. |
| `cassto.brownianDecay` | `false` | `true`, `false` | Linearly anneals Brownian step size from `brownianDecayFrom` to `brownianDecayTo`. |
| `cassto.brownianDecayFrom` | `8.0` | Float | Starting step size for Brownian decay schedule. |
| `cassto.brownianDecayTo` | `1.0` | Float | Final step size for Brownian decay schedule. |
| `evals` | `3000` | `3000`, `10000`, `30000` | Maximum candidate schedule evaluation budget. |
| `seed` | `42` | Long | Random number generator seed for strict reproducibility. |
| `sameVmSwap` | `true` | `true`, `false` | Permits intra-VM task order swapping during Brownian moves. |

---

## 📊 Summary of Empirical Findings

Extensive statistical benchmarking (>600 simulation runs across Montage_100, CyberShake, and heterogeneous cloud environments) revealed the following scientific insights:

1. **4-Arm Ablation & Controller Isolation**:
   - The reported ~8.3% improvement of "Adaptive On" over default CASSTO was **not** caused by the online feedback controller.
   - When initial parameters are matched, **Adaptive does not outperform Fixed-4** ($p = 0.4825$).
   - The online controller’s primary function is **variance damping** (reduces IQR by up to 70%), not mean makespan optimization.
2. **$2 \times 2$ Factorial Two-Way ANOVA ($b \in \{1, 4\} \times \alpha \in \{2.0, 2.5\}$)**:
   - **Brownian Step Size ($b$)**: Explains **28.78% of total makespan variance** ($F = 47.67, p = 2.86 \times 10^{-10}$).
   - **Lévy Exponent ($\alpha$)**: Explains only **0.91% of variance** ($F = 1.50, p = 0.222$, statistically insignificant).
   - **Interaction ($b \times \alpha$)**: Negligible ($0.27\%$, $p = 0.506$).
3. **Evaluation Budget Scaling (3,000 $\to$ 10,000 $\to$ 30,000 evaluations)**:
   - Step size acts as both a **convergence speed lever** and a **topological barrier-crossing operator**.
   - Step-1 drops from 81.19s down to 60.66s at 30k evals, but **Step-4 remains significantly superior** (58.16s, $p = 3.52 \times 10^{-7}$).
4. **Genetic Algorithm vs. CASSTO**:
   - Standard **GA with crossover recombination consistently outperforms CASSTO** across all evaluation tiers (e.g., at 30k evals, GA reaches **54.16s** with best individual **48.89s**, beating CASSTO's 58.16s with $p = 6.52 \times 10^{-9}$).

---

## 📂 Project Structure

```plaintext
WorkflowSim-1/
├── config/
│   └── dax/                         # Real-world scientific workflow DAG datasets (XML format)
│       ├── Montage_{25..1000}.xml   # Astronomy workflow
│       ├── CyberShake_{30..1000}.xml# Earthquake science workflow
│       ├── Epigenomics_{24..997}.xml# Genome sequencing workflow
│       ├── Inspiral_{30..1000}.xml  # Gravitational physics (LIGO) workflow
│       └── Sipht_{30..1000}.xml     # Bioinformatics workflow
├── sources/
│   └── org/workflowsim/
│       ├── planning/                # Core planning algorithms
│       │   ├── CASSTOPlanningAlgorithm.java  # Audited & enhanced CASSTO
│       │   ├── GAPlanningAlgorithm.java       # Standard Genetic Algorithm
│       │   ├── HICAPlanningAlgorithm.java     # Hybrid Imperialist Competitive Algorithm
│       │   ├── HEFTPlanningAlgorithm.java     # List heuristic baseline
│       │   ├── DHEFTPlanningAlgorithm.java    # Dynamic list heuristic
│       │   ├── OrderAwarePlanningBase.java    # Shared evaluation & DAG engine
│       │   └── RandomPlanningAlgorithm.java   # Random baseline
│       ├── scheduling/              # VM scheduling handlers
│       └── WorkflowEngine.java      # Workflow execution simulation core
├── examples/
│   └── org/workflowsim/examples/
│       ├── CASSTO_1.java            # CASSTO driver
│       ├── GA_1.java                # GA driver
│       └── HICA.java                # HICA driver
├── tmp/analysis/                    # Automated Python benchmarking & statistical suite
│   ├── auto_runner.py               # Parallel simulation runner with metrics parser
│   ├── run_ablation_arms.py         # 4-arm controller ablation study
│   ├── run_factorial_2x2.py         # 2x2 Factorial ANOVA (Brownian Step x Levy Alpha)
│   ├── run_eval_scaling.py          # Budget scaling evaluation (3k, 10k, 30k evals)
│   ├── run_ga_eval_scaling.py       # GA vs CASSTO scaling comparison
│   ├── run_decay_vs_fixed4.py       # Simulated annealing step decay study
│   └── statistical_tests.py         # Mann-Whitney U, Wilcoxon, Cliff's delta, Two-way ANOVA
├── lib/                             # Required runtime JAR dependencies
│   ├── commons-math3-3.2.jar
│   ├── flanagan.jar
│   └── jdom-2.0.0.jar
└── README.md
```

---

## 🛠️ Build & Execution Guide

### 1. Compilation
Compile all Java sources and examples into the `bin/` directory:
```bash
# On Windows (PowerShell):
Get-ChildItem -Recurse sources/*.java, examples/*.java | Resolve-Path -Relative | Out-File -Encoding ascii sources.txt
javac -cp "lib/*;classes;bin" -d bin "@sources.txt"

# On Linux/macOS:
find sources examples -name "*.java" > sources.txt
javac -cp "lib/*:classes:bin" -d bin @sources.txt
```

### 2. Running Simulations Directly via Java
```bash
# Run CASSTO with fixed Brownian step = 4 and 10,000 evaluations:
java -Dseed=101 -Dcassto.brownianMoves=4 -Devals=10000 -cp "bin;lib/*" org.workflowsim.examples.CASSTO_1

# Run Genetic Algorithm baseline:
java -Dseed=101 -Devals=10000 -cp "bin;lib/*" org.workflowsim.examples.GA_1
```

### 3. Automated Parallel Python Test Suite
The repository includes an automated parallel execution and statistical analysis harness in `tmp/analysis/`:
```bash
# Run 30 paired simulations in parallel (6 workers):
python tmp/analysis/auto_runner.py --runs 30 --seed-start 101 --algos CASSTO --workers 6 --prop cassto.brownianMoves=4

# Run the 2x2 Factorial ANOVA study:
python tmp/analysis/run_factorial_2x2.py --seed-start 101 --runs 30

# Run evaluation scaling comparison (3k, 10k, 30k):
python tmp/analysis/run_eval_scaling.py --seed-start 101 --runs 30
```

---

## 📖 Citation & References

```bibtex
@article{hameed2026cassto,
  title={CASSTO: a bio-inspired metaheuristic for QoS-oriented scientific workflow scheduling in cloud computing},
  author={Hameed, R. and others},
  journal={The Journal of Supercomputing},
  year={2026}
}

@article{chen2012workflowsim,
  title={WorkflowSim: A Toolkit for Simulating Scientific Workflows in Distributed Environments},
  author={Chen, Weiwei and Deelman, Ewa},
  journal={2012 IEEE 8th International Conference on E-Science},
  pages={1--8},
  year={2012},
  publisher={IEEE}
}
```

---

## 📄 License
Distributed under the Apache License Version 2.0. See [`license.txt`](license.txt) and [`license-workflowsim.txt`](license-workflowsim.txt) for details.
