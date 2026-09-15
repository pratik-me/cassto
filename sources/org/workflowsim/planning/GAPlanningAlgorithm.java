package org.workflowsim.planning;

import java.util.*;

import org.cloudbus.cloudsim.Log;
import org.workflowsim.CondorVM;
import org.workflowsim.Task;

/**
 * GA: Genetic Algorithm for workflow scheduling, matching Algorithm 2 and
 * Section 5.1's hyperparameters from the paper: population size = 30,
 * crossover probability p_c = 0.8, mutation probability p_m = 0.05, max
 * generations = 100, tournament selection (size 3), elitism.
 *
 * Uses the SAME DAG traversal and fitness function as
 * CASSTOPlanningAlgorithm / HICAPlanningAlgorithm, so makespan/cost/
 * utilization results are directly comparable across all three.
 *
 * Representation note: like CASSTO and HICA above, this GA operates on a
 * fixed, shared topological task order (computed once via DFS) and evolves
 * only the VM assignment per task. Precedence is enforced by that fixed
 * order plus WorkflowSim's own DAG structure, not by the chromosome, so
 * crossover/mutation never need a separate "repair" step to fix broken
 * dependencies (Algorithm 2's option (b), task-VM reassignment). If you
 * want literal task-order evolution too (Algorithm 2's option (a), order
 * crossover / adjacent-swap mutation), that would require also encoding
 * and repairing a topological permutation per individual.
 */
public class GAPlanningAlgorithm extends BasePlanningAlgorithm {

    // ---- GA hyperparameters, from paper Sec. 5.1 ----
    private static final int POP_SIZE = 30;
    private static final int MAX_GENERATIONS = 100;
    private static final double CROSSOVER_PROB = 0.8;
    private static final double MUTATION_PROB = 0.05;
    private static final int TOURNAMENT_SIZE = 3;

    // ---- Fitness weights, identical to CASSTO/HICA for fair comparison ----
    private static final double W1_MAKESPAN = 0.5;
    private static final double W2_COST = 0.3;
    private static final double W3_UTIL = 0.2;
    private static final double PRICE_PER_MIPS_SECOND = 0.001;

    private List<Task> topoOrder;
    private final Random rnd = new Random();

    @Override
    public void run() {
        topoOrder = topologicalSort();
        int n = topoOrder.size();
        int m = getVmList().size();

        Log.printLine("GA planning: " + n + " tasks, " + m + " VMs.");

        // ---- 1. Initialization + fitness evaluation ----
        int[][] population = new int[POP_SIZE][n];
        double[] fitness = new double[POP_SIZE];
        for (int i = 0; i < POP_SIZE; i++) {
            for (int t = 0; t < n; t++) {
                population[i][t] = rnd.nextInt(m);
            }
            fitness[i] = evaluate(population[i]);
        }

        int bestIdx = bestOf(fitness);
        int[] best = population[bestIdx].clone();
        double bestFitness = fitness[bestIdx];

        // ---- 2. Generation loop ----
        for (int gen = 0; gen < MAX_GENERATIONS; gen++) {

            int[][] offspring = new int[POP_SIZE][];
            for (int i = 0; i < POP_SIZE; i += 2) {
                int[] parentA = tournamentSelect(population, fitness);
                int[] parentB = tournamentSelect(population, fitness);

                int[] childA;
                int[] childB;
                if (rnd.nextDouble() < CROSSOVER_PROB) {
                    int[][] children = crossover(parentA, parentB);
                    childA = children[0];
                    childB = children[1];
                } else {
                    childA = parentA.clone();
                    childB = parentB.clone();
                }

                mutate(childA, m);
                mutate(childB, m);

                offspring[i] = childA;
                if (i + 1 < POP_SIZE) {
                    offspring[i + 1] = childB;
                }
            }

            double[] offspringFitness = new double[POP_SIZE];
            for (int i = 0; i < POP_SIZE; i++) {
                offspringFitness[i] = evaluate(offspring[i]);
            }

            // ---- 3. Elitism: combine parents + offspring, keep the top POP_SIZE ----
            int[][] combined = new int[2 * POP_SIZE][];
            double[] combinedFitness = new double[2 * POP_SIZE];
            for (int i = 0; i < POP_SIZE; i++) {
                combined[i] = population[i];
                combinedFitness[i] = fitness[i];
                combined[POP_SIZE + i] = offspring[i];
                combinedFitness[POP_SIZE + i] = offspringFitness[i];
            }

            Integer[] order = sortIndicesByCost(combinedFitness);
            for (int i = 0; i < POP_SIZE; i++) {
                population[i] = combined[order[i]];
                fitness[i] = combinedFitness[order[i]];
            }

            if (fitness[0] < bestFitness) {
                bestFitness = fitness[0];
                best = population[0].clone();
            }

            if (gen % 10 == 0) {
                Log.printLine("GA generation " + gen + " best fitness = " + bestFitness);
            }
        }

        // ---- 4. Apply the best mapping found ----
        for (int t = 0; t < n; t++) {
            Task task = topoOrder.get(t);
            CondorVM vm = (CondorVM) getVmList().get(best[t]);
            task.setVmId(vm.getId());
        }
        Log.printLine("GA planning finished, best fitness = " + bestFitness);
    }

    // ------------------------------------------------------------------
    // DAG + fitness — identical model to CASSTOPlanningAlgorithm / HICAPlanningAlgorithm
    // ------------------------------------------------------------------

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

            double execTime = task.getCloudletLength() / mips;
            double finish = readyTime + execTime;

            finishTime.put(task, finish);
            vmFreeAt[mapping[idx]] = finish;

            totalCost += execTime * PRICE_PER_MIPS_SECOND * mips;
            totalBusy += execTime;
        }

        double makespan = 0;
        for (double t : vmFreeAt) {
            makespan = Math.max(makespan, t);
        }

        double totalAvailable = makespan * m;
        double resourceUtil = totalAvailable > 0 ? totalBusy / totalAvailable : 0;

        return W1_MAKESPAN * makespan + W2_COST * totalCost + W3_UTIL * (1 - resourceUtil);
    }

    // ------------------------------------------------------------------
    // GA operators (Algorithm 2)
    // ------------------------------------------------------------------

    /** Tournament selection, size TOURNAMENT_SIZE. */
    private int[] tournamentSelect(int[][] population, double[] fitness) {
        int bestIdx = -1;
        double bestFit = Double.MAX_VALUE;
        for (int k = 0; k < TOURNAMENT_SIZE; k++) {
            int idx = rnd.nextInt(population.length);
            if (fitness[idx] < bestFit) {
                bestFit = fitness[idx];
                bestIdx = idx;
            }
        }
        return population[bestIdx];
    }

    /**
     * Two-point crossover over the VM-assignment chromosome. Since the task
     * order is fixed and shared across the whole population, this is
     * automatically precedence-preserving (no repair step needed).
     */
    private int[][] crossover(int[] parentA, int[] parentB) {
        int n = parentA.length;
        int[] childA = parentA.clone();
        int[] childB = parentB.clone();

        if (n < 2) {
            return new int[][]{childA, childB};
        }

        int p1 = rnd.nextInt(n);
        int p2 = rnd.nextInt(n);
        int lo = Math.min(p1, p2);
        int hi = Math.max(p1, p2);

        for (int t = lo; t <= hi; t++) {
            int tmp = childA[t];
            childA[t] = childB[t];
            childB[t] = tmp;
        }
        return new int[][]{childA, childB};
    }

    /** Mutation (Algorithm 2, option b): reassign a task to a different VM, gene-by-gene. */
    private void mutate(int[] individual, int vmCount) {
        for (int t = 0; t < individual.length; t++) {
            if (rnd.nextDouble() < MUTATION_PROB) {
                individual[t] = rnd.nextInt(vmCount);
            }
        }
    }

    // ------------------------------------------------------------------
    // small helpers
    // ------------------------------------------------------------------

    private int bestOf(double[] fitness) {
        int best = 0;
        for (int i = 1; i < fitness.length; i++) {
            if (fitness[i] < fitness[best]) {
                best = i;
            }
        }
        return best;
    }

    private Integer[] sortIndicesByCost(double[] fitness) {
        Integer[] idx = new Integer[fitness.length];
        for (int i = 0; i < idx.length; i++) {
            idx[i] = i;
        }
        Arrays.sort(idx, Comparator.comparingDouble(i -> fitness[i]));
        return idx;
    }
}