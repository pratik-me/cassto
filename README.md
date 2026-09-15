# CASSTO: QoS-Oriented Scientific Workflow Scheduling in Cloud Computing

An extension of the **WorkflowSim 1.0** toolkit implementing and benchmarking **CASSTO** (*Cloud-Aware Seal Search Task Optimization*) and comparative metaheuristic scheduling algorithms for scientific workflows in cloud computing environments.

---

## 📌 Overview & Academic Background

This repository provides the reference Java implementation and evaluation testbeds for the academic paper:

> **"CASSTO: a bio‑inspired metaheuristic for QoS‑oriented scientific workflow scheduling in cloud computing"**  
> *Hameed R. et al., Journal of Supercomputing (2026).*

Scientific workflow scheduling in cloud environments is an NP-hard optimization problem involving conflicting Quality of Service (QoS) criteria such as makespan, monetary execution cost, and resource utilization under strict DAG task precedence constraints. 

This project extends **WorkflowSim 1.0** with novel bio-inspired and evolutionary metaheuristics, allowing researchers and practitioners to seamlessly compare the performance of **CASSTO** against other metaheuristics (**HICA**, **GA**) and standard workflow heuristics (**HEFT**, **DHEFT**, **Random**).

---

## 🚀 Implemented Algorithms

| Algorithm | Type | Implementation Class | Description |
| :--- | :--- | :--- | :--- |
| **CASSTO** | Bio-inspired Metaheuristic | [`CASSTOPlanningAlgorithm.java`](sources/org/workflowsim/planning/CASSTOPlanningAlgorithm.java) | **Cloud-Aware Seal Search Task Optimization**: Uses Brownian search for local exploitation and Lévy flight jumps for global exploration, guided by population diversity metrics and task urgency (critical path). |
| **HICA** | Evolutionary Metaheuristic | [`HICAPlanningAlgorithm.java`](sources/org/workflowsim/planning/HICAPlanningAlgorithm.java) | **Hybrid Imperialist Competitive Algorithm**: Socio-politically inspired metaheuristic with imperialist assimilation, colony revolution, and inter-empire competition. |
| **GA** | Evolutionary Metaheuristic | [`GAPlanningAlgorithm.java`](sources/org/workflowsim/planning/GAPlanningAlgorithm.java) | **Genetic Algorithm**: Standard baseline featuring tournament selection ($k=3$), uniform crossover ($p_c=0.8$), mutation ($p_m=0.05$), and elitism. |
| **HEFT** | List-Scheduling Heuristic | [`HEFTPlanningAlgorithm.java`](sources/org/workflowsim/planning/HEFTPlanningAlgorithm.java) | **Heterogeneous Earliest Finish Time**: Deterministic DAG scheduling baseline prioritizing tasks via upward rank values ($rank_u$). |
| **DHEFT** | Dynamic Heuristic | [`DHEFTPlanningAlgorithm.java`](sources/org/workflowsim/planning/DHEFTPlanningAlgorithm.java) | **Dynamic HEFT**: Adaptive variation of HEFT for heterogeneous environments. |
| **Random** | Baseline | [`RandomPlanningAlgorithm.java`](sources/org/workflowsim/planning/RandomPlanningAlgorithm.java) | Uniformly distributes tasks randomly across available VMs. |

---

## 🎯 QoS Objectives & Multi-Objective Fitness Model

To ensure fair and rigorous benchmarking across CASSTO, HICA, and GA, all metaheuristics share an identical DAG traversal pipeline and weighted-sum fitness formulation:

$$\text{Fitness} = w_1 \cdot \text{Makespan}_{\text{norm}} + w_2 \cdot \text{Cost}_{\text{norm}} + w_3 \cdot (1 - \text{ResourceUtilization})$$

- **Makespan**: Maximum completion time across all workflow tasks.
- **Cost**: VM computation cost based on execution duration and VM processing capacity (MIPS-rate pricing model).
- **Resource Utilization**: Ratio of busy VM execution time to total VM active leasing time.
- **Precedence Integrity**: Topological ordering guarantees 100% valid task execution without precedence violations.

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
│       │   ├── CASSTOPlanningAlgorithm.java
│       │   ├── HICAPlanningAlgorithm.java
│       │   ├── GAPlanningAlgorithm.java
│       │   ├── HEFTPlanningAlgorithm.java
│       │   ├── DHEFTPlanningAlgorithm.java
│       │   └── RandomPlanningAlgorithm.java
│       ├── scheduling/              # Scheduling & VM mapping handlers
│       ├── WorkflowPlanner.java     # Planner orchestrator
│       └── WorkflowEngine.java      # Workflow execution engine
├── examples/
│   └── org/workflowsim/examples/
│       ├── CASSTO_1.java            # CASSTO execution example & benchmark runner
│       ├── HICA.java                # HICA execution example
│       ├── GA_1.java                # GA execution example
│       └── planning/                # HEFT & DHEFT example runners
├── lib/                             # Required runtime JAR libraries
│   ├── commons-math3-3.2.jar
│   ├── flanagan.jar
│   └── jdom-2.0.0.jar
└── README.md
```

---

## 🛠️ Prerequisites & Setup

- **Java Development Kit (JDK)**: Java 8 or newer (JDK 8 / 11 / 17 / 21+).
- **IDE (Recommended)**: Eclipse, IntelliJ IDEA, VS Code, or NetBeans.

### Adding External Libraries
Ensure all JAR files in `lib/` are present in your Java project's Build Path:
1. `lib/commons-math3-3.2.jar`
2. `lib/flanagan.jar`
3. `lib/jdom-2.0.0.jar`

---

## ⚡ Quickstart & Running Comparisons

You can execute the example classes directly to run simulations and compare algorithm metrics.

### 1. Running CASSTO
Run [`CASSTO_1.java`](examples/org/workflowsim/examples/CASSTO_1.java):
```java
// Configure target workflow DAX file:
String daxPath = "config/dax/Montage_1000.xml";

// Set planning algorithm:
Parameters.PlanningAlgorithm pln_method = Parameters.PlanningAlgorithm.CASSTO;
Parameters.SchedulingAlgorithm sch_method = Parameters.SchedulingAlgorithm.STATIC;
```

### 2. Running Comparative Baselines
- **HICA**: Run [`HICA.java`](examples/org/workflowsim/examples/HICA.java)
- **GA**: Run [`GA_1.java`](examples/org/workflowsim/examples/GA_1.java)
- **HEFT**: Run [`HEFTPlanningAlgorithmExample1.java`](examples/org/workflowsim/examples/planning/HEFTPlanningAlgorithmExample1.java)
- **DHEFT**: Run [`DHEFTPlanningAlgorithmExample1.java`](examples/org/workflowsim/examples/planning/DHEFTPlanningAlgorithmExample1.java)

### 3. Fair Benchmarking Setup
To maintain consistency during comparative evaluation:
- Use the **same DAX file** (e.g., `Montage_1000.xml`, `CyberShake_1000.xml`).
- Maintain the **same VM pool** (Default: 20 heterogeneous VMs with cyclic 1000 / 2000 / 3000 MIPS).
- Use `STATIC` scheduling mode so the planner's VM mappings are preserved during simulation execution.

---

## 📊 Sample Output

When running `CASSTO_1.java`, execution logs and performance metrics are displayed in the console:

```plaintext
CASSTO planning: 1000 tasks, 20 VMs.
CASSTO iter 1 best fitness = 0.4812
CASSTO iter 25 best fitness = 0.3951
CASSTO iter 50 best fitness = 0.3542
CASSTO iter 100 best fitness = 0.3128
CASSTO planning finished, best fitness = 0.3128

========== CASSTO RESULTS ==========
Number of jobs: 1000
Makespan: 218.45
```

---

## 📖 Citation & References

If you use this implementation in your research, please cite the following papers:

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
This software is distributed under the Apache License Version 2.0. See [`license.txt`](license.txt) and [`license-workflowsim.txt`](license-workflowsim.txt) for full details.
