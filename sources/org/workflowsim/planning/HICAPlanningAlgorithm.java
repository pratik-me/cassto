package org.workflowsim.planning;

import java.util.*;

import org.cloudbus.cloudsim.Log;
import org.workflowsim.CondorVM;
import org.workflowsim.Task;

/**
 * HICA: Hybrid Imperialist Competitive Algorithm for workflow scheduling.
 *
 * Reference: Hu L., Wu X., Che X. (2025) "HICA: a hybrid scientific workflow
 * scheduling algorithm for symmetric homogeneous resource cloud environments."
 * Symmetry 17(2):280.
 *
 * This planning algorithm uses the SAME DAG traversal and fitness function
 * (Eq. 1, 2, 3, 11, 12-style weighted sum) as CASSTOPlanningAlgorithm, so
 * makespan/cost/utilization numbers from the two are directly comparable.
 * Only the search mechanism differs: instead of Levy/Brownian moves over a
 * single population, ICA splits the population into empires (one
 * imperialist + several colonies each) that assimilate, revolt, and compete
 * for each other's colonies until (ideally) one empire dominates.
 *
 * Discretization note: classic ICA moves colonies toward their imperialist
 * in continuous space (x_new = x + beta * d). Since our search space is a
 * discrete task->VM assignment, "assimilation" here is implemented as: each
 * task's VM assignment is copied from the imperialist with probability
 * ASSIMILATION_PROB. This is a standard discretization used in permutation-
 * /assignment-based ICA implementations.
 */
public class HICAPlanningAlgorithm extends BasePlanningAlgorithm {

    // ---- ICA hyperparameters ----
    private static final int NUM_COUNTRIES = 30;         // matches CASSTO's population size
    private static final int NUM_IMPERIALISTS = 6;        // ~20% of countries become imperialists
    private static final int MAX_ITER = 100;               // matches CASSTO's iteration count
    private static final double ASSIMILATION_PROB = 0.6;   // chance a gene moves toward the imperialist
    private static final double REVOLUTION_RATE = 0.1;     // chance a gene mutates randomly
    private static final double ZETA = 0.1;                 // weight of mean colony cost in empire total cost

    // ---- Fitness weights, identical to CASSTOPlanningAlgorithm for fair comparison ----
    private static final double W1_MAKESPAN = 0.5;
    private static final double W2_COST = 0.3;
    private static final double W3_UTIL = 0.2;
    private static final double PRICE_PER_MIPS_SECOND = 0.001;

    private List<Task> topoOrder;
    private final Random rnd = new Random();

    private static class Empire {
        int imperialistIdx;
        List<Integer> colonyIdx = new ArrayList<>();
    }

    @Override
    public void run() {
        topoOrder = topologicalSort();
        int n = topoOrder.size();
        int m = getVmList().size();

        Log.printLine("HICA planning: " + n + " tasks, " + m + " VMs.");

        // ---- 1. Initialize countries (candidate task->VM mappings) ----
        int[][] countries = new int[NUM_COUNTRIES][n];
        double[] cost = new double[NUM_COUNTRIES];
        for (int i = 0; i < NUM_COUNTRIES; i++) {
            for (int t = 0; t < n; t++) {
                countries[i][t] = rnd.nextInt(m);
            }
            cost[i] = evaluate(countries[i]);
        }

        // ---- 2. Select imperialists: the NUM_IMPERIALISTS lowest-cost countries ----
        Integer[] order = sortIndicesByCost(cost);
        List<Empire> empires = new ArrayList<>();
        for (int k = 0; k < NUM_IMPERIALISTS; k++) {
            Empire e = new Empire();
            e.imperialistIdx = order[k];
            empires.add(e);
        }

        // ---- 3. Distribute remaining countries as colonies, proportional to imperialist power ----
        double[] impCost = new double[NUM_IMPERIALISTS];
        for (int k = 0; k < NUM_IMPERIALISTS; k++) {
            impCost[k] = cost[empires.get(k).imperialistIdx];
        }
        double maxImpCost = max(impCost);
        double[] power = new double[NUM_IMPERIALISTS];
        double powerSum = 0;
        for (int k = 0; k < NUM_IMPERIALISTS; k++) {
            power[k] = maxImpCost - impCost[k] + 1e-9; // lower cost => higher power
            powerSum += power[k];
        }

        List<Integer> remaining = new ArrayList<>();
        for (int k = NUM_IMPERIALISTS; k < NUM_COUNTRIES; k++) {
            remaining.add(order[k]);
        }
        Collections.shuffle(remaining, rnd);

        int idx = 0;
        for (int k = 0; k < NUM_IMPERIALISTS; k++) {
            int numColonies = (int) Math.round((power[k] / powerSum) * remaining.size());
            for (int c = 0; c < numColonies && idx < remaining.size(); c++, idx++) {
                empires.get(k).colonyIdx.add(remaining.get(idx));
            }
        }
        while (idx < remaining.size()) { // any leftover countries join the strongest empire
            empires.get(0).colonyIdx.add(remaining.get(idx));
            idx++;
        }

        // ---- 4. Main ICA loop: assimilation, revolution, competition ----
        for (int iter = 0; iter < MAX_ITER && empires.size() > 1; iter++) {

            for (Empire empire : empires) {
                int[] imperialist = countries[empire.imperialistIdx];
                List<Integer> colonies = new ArrayList<>(empire.colonyIdx); // snapshot, we mutate below

                for (int colIdx : colonies) {
                    assimilate(countries[colIdx], imperialist);
                    revolt(countries[colIdx], m);
                    cost[colIdx] = evaluate(countries[colIdx]);

                    // a colony that becomes stronger than its imperialist takes over the empire
                    if (cost[colIdx] < cost[empire.imperialistIdx]) {
                        int former = empire.imperialistIdx;
                        empire.imperialistIdx = colIdx;
                        empire.colonyIdx.remove((Integer) colIdx);
                        empire.colonyIdx.add(former);
                    }
                }
            }

            imperialisticCompetition(empires, cost);

            if (iter % 10 == 0) {
                Empire strongest = strongestEmpire(empires, cost);
                Log.printLine("HICA iter " + iter + " best cost = " + cost[strongest.imperialistIdx]
                        + " empires remaining = " + empires.size());
            }
        }

        // ---- 5. Apply the best mapping found ----
        Empire best = strongestEmpire(empires, cost);
        int[] bestMapping = countries[best.imperialistIdx];
        for (int t = 0; t < n; t++) {
            Task task = topoOrder.get(t);
            CondorVM vm = (CondorVM) getVmList().get(bestMapping[t]);
            task.setVmId(vm.getId());
        }
        Log.printLine("HICA planning finished, best cost = " + cost[best.imperialistIdx]);
    }

    // ------------------------------------------------------------------
    // DAG + fitness — identical model to CASSTOPlanningAlgorithm
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
    // ICA-specific operators
    // ------------------------------------------------------------------

    /** Assimilation: colony genes move toward the imperialist's genes. */
    private void assimilate(int[] colony, int[] imperialist) {
        for (int t = 0; t < colony.length; t++) {
            if (rnd.nextDouble() < ASSIMILATION_PROB) {
                colony[t] = imperialist[t];
            }
        }
    }

    /** Revolution: small chance of a fully random gene, to preserve diversity. */
    private void revolt(int[] colony, int vmCount) {
        for (int t = 0; t < colony.length; t++) {
            if (rnd.nextDouble() < REVOLUTION_RATE) {
                colony[t] = rnd.nextInt(vmCount);
            }
        }
    }

    /** Total empire cost = imperialist cost + zeta * mean colony cost (standard ICA formula). */
    private double empireTotalCost(Empire empire, double[] cost) {
        double total = cost[empire.imperialistIdx];
        if (!empire.colonyIdx.isEmpty()) {
            double meanColonyCost = 0;
            for (int c : empire.colonyIdx) {
                meanColonyCost += cost[c];
            }
            meanColonyCost /= empire.colonyIdx.size();
            total += ZETA * meanColonyCost;
        }
        return total;
    }

    private Empire strongestEmpire(List<Empire> empires, double[] cost) {
        Empire best = empires.get(0);
        double bestCost = empireTotalCost(best, cost);
        for (Empire e : empires) {
            double c = empireTotalCost(e, cost);
            if (c < bestCost) {
                bestCost = c;
                best = e;
            }
        }
        return best;
    }

    private Empire weakestEmpire(List<Empire> empires, double[] cost) {
        Empire worst = empires.get(0);
        double worstCost = empireTotalCost(worst, cost);
        for (Empire e : empires) {
            double c = empireTotalCost(e, cost);
            if (c > worstCost) {
                worstCost = c;
                worst = e;
            }
        }
        return worst;
    }

    /**
     * Imperialistic competition: the weakest colony of the weakest empire is
     * handed to a stronger empire, chosen by roulette wheel weighted by
     * relative power. If the weakest empire has no colonies left, it
     * collapses and its imperialist becomes a colony of the strongest empire.
     */
    private void imperialisticCompetition(List<Empire> empires, double[] cost) {
        if (empires.size() < 2) {
            return;
        }

        Empire weakest = weakestEmpire(empires, cost);
        if (weakest.colonyIdx.isEmpty()) {
            empires.remove(weakest);
            if (!empires.isEmpty()) {
                strongestEmpire(empires, cost).colonyIdx.add(weakest.imperialistIdx);
            }
            return;
        }

        int worstColony = weakest.colonyIdx.get(0);
        double worstColonyCost = cost[worstColony];
        for (int c : weakest.colonyIdx) {
            if (cost[c] > worstColonyCost) {
                worstColonyCost = cost[c];
                worstColony = c;
            }
        }

        double[] totalCosts = new double[empires.size()];
        for (int i = 0; i < empires.size(); i++) {
            totalCosts[i] = empireTotalCost(empires.get(i), cost);
        }
        double maxTotal = max(totalCosts);

        double[] weights = new double[empires.size()];
        double sumWeights = 0;
        for (int i = 0; i < empires.size(); i++) {
            weights[i] = (empires.get(i) == weakest) ? 0 : (maxTotal - totalCosts[i] + 1e-9);
            sumWeights += weights[i];
        }

        double r = rnd.nextDouble() * sumWeights;
        double acc = 0;
        Empire winner = weakest;
        for (int i = 0; i < empires.size(); i++) {
            acc += weights[i];
            if (r <= acc && empires.get(i) != weakest) {
                winner = empires.get(i);
                break;
            }
        }

        if (winner != weakest) {
            weakest.colonyIdx.remove((Integer) worstColony);
            winner.colonyIdx.add(worstColony);
        }
    }

    private Integer[] sortIndicesByCost(double[] cost) {
        Integer[] idx = new Integer[cost.length];
        for (int i = 0; i < idx.length; i++) {
            idx[i] = i;
        }
        Arrays.sort(idx, Comparator.comparingDouble(i -> cost[i]));
        return idx;
    }

    private double max(double[] arr) {
        double m = arr[0];
        for (double v : arr) {
            m = Math.max(m, v);
        }
        return m;
    }
}