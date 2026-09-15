package org.workflowsim.planning;

import java.util.*;

import org.cloudbus.cloudsim.Log;
import org.workflowsim.CondorVM;
import org.workflowsim.Task;

/**
 * CASSTO: Cloud-Aware Seal Search Task Optimization.
 *
 * Implements the algorithm described in:
 * Hameed R. et al., "CASSTO: a bio-inspired metaheuristic for QoS-oriented
 * scientific workflow scheduling in cloud computing", J. Supercomputing (2026).
 *
 * This is a *planning* algorithm: it runs once, before execution, over the
 * whole DAG (getTaskList()) and the whole VM pool (getVmList()), exactly as
 * Section 4.3 of the paper assumes ("the entire task dependency network is
 * known before the scheduling").
 *
 * Population-based search alternates between:
 *  - Brownian phase (local exploitation): small perturbations, used when
 *    population diversity D < deltaThreshold.
 *  - Levy phase (global exploration): large jumps sized by a Levy-stable
 *    step, prioritizing "urgent" (critical-path) tasks, used when D >= delta.
 *
 * Fitness follows Eq. (12): weighted sum of makespan, cost, and
 * (1 - resource utilization). The energy term from Eq. (12) is omitted here
 * since WorkflowSim has no built-in energy model; set W3 to weight resource
 * utilization instead, or plug in your own energy estimate if you have one.
 */
public class CASSTOPlanningAlgorithm extends BasePlanningAlgorithm {

    // ---- CASSTO hyperparameters, taken from the paper's Sec. 5.2 ----
    private static final int POP_SIZE = 30;
    private static final int MAX_ITER = 100;
    private static final double LEVY_LAMBDA = 1.5;
    private static final double DIVERSITY_THRESHOLD = 0.25;

    // ---- Fitness weights (Eq. 12), must sum to 1 ----
    private static final double W1_MAKESPAN = 0.5;
    private static final double W2_COST = 0.3;
    private static final double W3_UTIL = 0.2;

    // Simple cost model: price is proportional to VM speed (tune to your VM
    // pricing scheme, or read a real per-VM price if your CondorVM has one).
    private static final double PRICE_PER_MIPS_SECOND = 0.001;

    private List<Task> topoOrder;
    private Map<Task, Boolean> urgent;
    private final Random rnd = new Random();

    @Override
    public void run() {
        topoOrder = topologicalSort();
        urgent = computeUrgency(topoOrder);

        int n = topoOrder.size();
        int m = getVmList().size();

        Log.printLine("CASSTO planning: " + n + " tasks, " + m + " VMs.");

        int[][] population = new int[POP_SIZE][n];
        double[] fitness = new double[POP_SIZE];

        // ---- 1. Initialization + fitness evaluation ----
        for (int i = 0; i < POP_SIZE; i++) {
            for (int t = 0; t < n; t++) {
                population[i][t] = rnd.nextInt(m);
            }
            fitness[i] = evaluate(population[i]);
        }

        int bestIdx = bestOf(fitness);
        int[] best = population[bestIdx].clone();
        double bestFitness = fitness[bestIdx];

        // ---- 2. Main search loop ----
        for (int iter = 0; iter < MAX_ITER; iter++) {
            double diversity = populationDiversity(population);

            for (int i = 0; i < POP_SIZE; i++) {
                int[] candidate = population[i].clone();

                if (diversity < DIVERSITY_THRESHOLD) {
                    brownianMove(candidate, m);   // local exploitation
                } else {
                    levyMove(candidate, m);        // global exploration
                }

                double candidateFitness = evaluate(candidate);
                if (candidateFitness < fitness[i]) {
                    population[i] = candidate;
                    fitness[i] = candidateFitness;
                }
                if (fitness[i] < bestFitness) {
                    bestFitness = fitness[i];
                    best = population[i].clone();
                }
            }

            if (iter % 10 == 0) {
                Log.printLine("CASSTO iter " + iter + " best fitness = " + bestFitness);
            }
        }

        // ---- 3. Apply the best mapping found ----
        for (int t = 0; t < n; t++) {
            Task task = topoOrder.get(t);
            CondorVM vm = (CondorVM) getVmList().get(best[t]);
            task.setVmId(vm.getId());
        }
        Log.printLine("CASSTO planning finished, best fitness = " + bestFitness);
    }

    // ------------------------------------------------------------------
    // DAG handling
    // ------------------------------------------------------------------

    /** DFS-based topological sort using each Task's parent list. */
    private List<Task> topologicalSort() {
        List<Task> order = new ArrayList<>();
        Set<Task> visited = new HashSet<>();
        for (Object o : getTaskList()) {
            visitTask((Task) o, visited, order);
        }
        return order;
    }

    private void visitTask(Task t, Set<Task> visited, List<Task> order) {
        if (visited.contains(t)) {
            return;
        }
        visited.add(t);
        for (Object p : t.getParentList()) {
            visitTask((Task) p, visited, order);
        }
        order.add(t);
    }

    /**
     * Urgency indicator U_i (Eq. 6): a task is urgent if it lies on (or near)
     * the critical path, approximated here as the longest upstream path
     * length in million instructions.
     */
    private Map<Task, Boolean> computeUrgency(List<Task> order) {
        Map<Task, Double> longestPathTo = new HashMap<>();
        double maxPath = 0;

        for (Task t : order) {
            double best = t.getCloudletLength();
            for (Object p : t.getParentList()) {
                Task parent = (Task) p;
                best = Math.max(best, longestPathTo.get(parent) + t.getCloudletLength());
            }
            longestPathTo.put(t, best);
            maxPath = Math.max(maxPath, best);
        }

        Map<Task, Boolean> urgencyMap = new HashMap<>();
        double criticalThreshold = 0.8 * maxPath; // within 80% of the critical path
        for (Task t : order) {
            urgencyMap.put(t, longestPathTo.get(t) >= criticalThreshold);
        }
        return urgencyMap;
    }

    // ------------------------------------------------------------------
    // Fitness evaluation (Eqs. 1, 2, 3, 11, 12)
    // ------------------------------------------------------------------

    private double evaluate(int[] mapping) {
        int m = getVmList().size();
        double[] vmFreeAt = new double[m];
        Map<Task, Double> finishTime = new HashMap<>();
        double totalCost = 0;
        double totalBusy = 0;

        for (int idx = 0; idx < topoOrder.size(); idx++) {
            Task task = topoOrder.get(idx);
            CondorVM vm = (CondorVM) getVmList().get(mapping[idx]);
            double mips = vm.getMips();

            double readyTime = vmFreeAt[mapping[idx]];
            for (Object p : task.getParentList()) {
                Task parent = (Task) p;
                readyTime = Math.max(readyTime, finishTime.get(parent));
            }

            double execTime = task.getCloudletLength() / mips; // Eq. (1)
            double finish = readyTime + execTime;

            finishTime.put(task, finish);
            vmFreeAt[mapping[idx]] = finish;

            totalCost += execTime * PRICE_PER_MIPS_SECOND * mips; // proxy for Eq. (3)
            totalBusy += execTime;
        }

        double makespan = 0;
        for (double t : vmFreeAt) {
            makespan = Math.max(makespan, t); // Eq. (2)
        }

        double totalAvailable = makespan * m;
        double resourceUtil = totalAvailable > 0 ? totalBusy / totalAvailable : 0; // Eq. (11)

        return W1_MAKESPAN * makespan + W2_COST * totalCost + W3_UTIL * (1 - resourceUtil);
    }

    // ------------------------------------------------------------------
    // Population diversity + move operators (Eq. 5)
    // ------------------------------------------------------------------

    private double populationDiversity(int[][] population) {
        int popSize = population.length;
        int n = population[0].length;
        double totalDist = 0;
        int pairs = 0;

        for (int i = 0; i < popSize; i++) {
            for (int j = i + 1; j < popSize; j++) {
                int diff = 0;
                for (int t = 0; t < n; t++) {
                    if (population[i][t] != population[j][t]) {
                        diff++;
                    }
                }
                totalDist += (double) diff / n;
                pairs++;
            }
        }
        return pairs > 0 ? totalDist / pairs : 0;
    }

    /** Brownian phase: small, local perturbations (a handful of tasks nudged to a nearby VM). */
    private void brownianMove(int[] candidate, int vmCount) {
        int n = candidate.length;
        int moves = Math.max(1, n / 20);
        for (int k = 0; k < moves; k++) {
            int t = rnd.nextInt(n);
            int delta = rnd.nextBoolean() ? 1 : -1;
            candidate[t] = ((candidate[t] + delta) % vmCount + vmCount) % vmCount;
        }
    }

    /** Levy phase: large jumps sized by a Levy-stable step, urgent tasks reassigned first. */
    private void levyMove(int[] candidate, int vmCount) {
        int n = candidate.length;
        double step = levySample(LEVY_LAMBDA);
        int moves = Math.min(n, Math.max(1, (int) Math.ceil(Math.abs(step))));

        List<Task> byUrgency = new ArrayList<>(topoOrder);
        byUrgency.sort((a, b) -> Boolean.compare(!urgent.get(a), !urgent.get(b)));

        for (int k = 0; k < moves; k++) {
            Task task = byUrgency.get(k % byUrgency.size());
            int idx = topoOrder.indexOf(task);
            candidate[idx] = rnd.nextInt(vmCount);
        }
    }

    /** Mantegna's algorithm for sampling a symmetric Levy-stable step size. */
    private double levySample(double lambda) {
        double sigma = Math.pow(
                (gamma(1 + lambda) * Math.sin(Math.PI * lambda / 2))
                        / (gamma((1 + lambda) / 2) * lambda * Math.pow(2, (lambda - 1) / 2)),
                1 / lambda);
        double u = rnd.nextGaussian() * sigma;
        double v = rnd.nextGaussian();
        return u / Math.pow(Math.abs(v), 1 / lambda);
    }

    /** Lanczos approximation of the Gamma function (accurate enough for lambda in (1,3]). */
    private double gamma(double x) {
        double[] g = {676.5203681218851, -1259.1392167224028, 771.32342877765313,
                -176.61502916214059, 12.507343278686905, -0.13857109526572012,
                9.9843695780195716e-6, 1.5056327351493116e-7};
        if (x < 0.5) {
            return Math.PI / (Math.sin(Math.PI * x) * gamma(1 - x));
        }
        x -= 1;
        double a = 0.99999999999980993;
        double t = x + 7.5;
        for (int i = 0; i < g.length; i++) {
            a += g[i] / (x + i + 1);
        }
        return Math.sqrt(2 * Math.PI) * Math.pow(t, x + 0.5) * Math.exp(-t) * a;
    }

    private int bestOf(double[] fitness) {
        int best = 0;
        for (int i = 1; i < fitness.length; i++) {
            if (fitness[i] < fitness[best]) {
                best = i;
            }
        }
        return best;
    }
}