package org.workflowsim.planning;

import java.util.*;

import org.cloudbus.cloudsim.Log;
import org.workflowsim.CondorVM;
import org.workflowsim.Task;

public class GAPlanningAlgorithm extends BasePlanningAlgorithm {

    // -- GA hyperparameters, from paper Sec. 5.1 --
    private static final int POP_SIZE = 30;
    private static final int MAX_GENERATIONS = 100;
    private static final double CROSSOVER_PROB = 0.8;
    private static final double MUTATION_PROB = 0.05;
    private static final int TOURNAMENT_SIZE = 3;

    // -- Fitness weights --
    private static final double W1_MAKESPAN = 0.5;
    private static final double W2_COST = 0.3;
    private static final double W3_UTIL = 0.2;
    private static final double PRICE_PER_MIPS_SECOND = 0.001;

    private double refMakespan = 1.0;
    private double refCost = 1.0;

    private List<Task> topoOrder;
    private static final long SEED = Long.getLong("seed", 42L);
    private final Random rnd = new Random(SEED);

    @Override
    public void run() {
        topoOrder = topologicalSort();
        int n = topoOrder.size();
        int m = getVmList().size();

        Log.printLine("GA planning: " + n + " tasks, " + m + " VMs.");

        // -- 1. Initialization + reference normalization baselines (Eq. 4) --
        int[][] population = new int[POP_SIZE][n];
        double[] fitness = new double[POP_SIZE];
        double sumMakespan = 0;
        double sumCost = 0;
        double[][] initMetrics = new double[POP_SIZE][3];
        for (int i = 0; i < POP_SIZE; i++) {
            for (int t = 0; t < n; t++) {
                population[i][t] = rnd.nextInt(m);
            }
            repair(population[i], m); // Algorithm Line 4: repair DAG feasibility
            initMetrics[i] = computeRawMetrics(population[i]);
            sumMakespan += initMetrics[i][0];
            sumCost += initMetrics[i][1];
        }
        refMakespan = sumMakespan / POP_SIZE;
        refCost = sumCost / POP_SIZE;

        for (int i = 0; i < POP_SIZE; i++) {
            fitness[i] = calcFitness(initMetrics[i]);
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

            // Algorithm Lines 26-30: Repair and Evaluation for each offspring
            double[] offspringFitness = new double[POP_SIZE];
            for (int i = 0; i < POP_SIZE; i++) {
                repair(offspring[i], m); // repair before evaluating
                offspringFitness[i] = evaluate(offspring[i]);
            }

            // -- 3. Elitism: combine parents + offspring --
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

        // -- 4. Apply the best mapping found --
        for (int t = 0; t < n; t++) {
            Task task = topoOrder.get(t);
            CondorVM vm = (CondorVM) getVmList().get(best[t]);
            task.setVmId(vm.getId());
        }
        Log.printLine("GA planning finished, best fitness = " + bestFitness);
    }

    // DAG + fitness
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

    private double[] computeRawMetrics(int[] mapping) {
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

        return new double[] { makespan, totalCost, resourceUtil };
    }

    private double calcFitness(double[] metrics) {
        double normMakespan = refMakespan > 0 ? metrics[0] / refMakespan : metrics[0];
        double normCost = refCost > 0 ? metrics[1] / refCost : metrics[1];
        double normUtil = 1.0 - metrics[2];

        return W1_MAKESPAN * normMakespan + W2_COST * normCost + W3_UTIL * normUtil;
    }

    private double evaluate(int[] mapping) {
        return calcFitness(computeRawMetrics(mapping));
    }

    // GA operators (Algorithm 2)
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

    // Two-point crossover over the VM-assignment chromosome.
    private int[][] crossover(int[] parentA, int[] parentB) {
        int n = parentA.length;
        int[] childA = parentA.clone();
        int[] childB = parentB.clone();

        if (n < 2) {
            return new int[][] { childA, childB };
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
        return new int[][] { childA, childB };
    }

    // Mutation (Algorithm 2, Lines 21-24)
    // For each gene, with probability p_m applies one of:
    //   (a) swap two adjacent tasks in valid topological order (DAG-feasible)
    //   (b) reassign task to a different VM
    private void mutate(int[] individual, int vmCount) {
        int n = individual.length;
        for (int t = 0; t < n; t++) {
            if (rnd.nextDouble() < MUTATION_PROB) {
                if (rnd.nextBoolean() && n > 1) {
                    // Option (a): swap with adjacent task if DAG-safe
                    int t2 = (t + 1 < n) ? t + 1 : t - 1;
                    Task task1 = topoOrder.get(t);
                    Task task2 = topoOrder.get(t2);
                    if (!isDirectlyRelated(task1, task2)) {
                        int tmp = individual[t];
                        individual[t] = individual[t2];
                        individual[t2] = tmp;
                    }
                } else {
                    // Option (b): reassign to a different VM
                    individual[t] = rnd.nextInt(vmCount);
                }
            }
        }
    }

    // Repair schedule (Algorithm 2 Lines 4 & 28): clamp VM indices to valid range.
    private void repair(int[] individual, int vmCount) {
        for (int t = 0; t < individual.length; t++) {
            if (individual[t] < 0 || individual[t] >= vmCount) {
                individual[t] = rnd.nextInt(vmCount);
            }
        }
    }

    // Returns true if a and b are directly related (parent-child) in the DAG.
    private boolean isDirectlyRelated(Task a, Task b) {
        for (Object p : b.getParentList()) {
            if (p == a) return true;
        }
        for (Object p : a.getParentList()) {
            if (p == b) return true;
        }
        return false;
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

    private Integer[] sortIndicesByCost(double[] fitness) {
        Integer[] idx = new Integer[fitness.length];
        for (int i = 0; i < idx.length; i++) {
            idx[i] = i;
        }
        Arrays.sort(idx, Comparator.comparingDouble(i -> fitness[i]));
        return idx;
    }
}