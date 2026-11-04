package org.workflowsim.planning;

import java.util.*;

import org.cloudbus.cloudsim.Log;
import org.workflowsim.CondorVM;
import org.workflowsim.FileItem;
import org.workflowsim.Task;
import org.workflowsim.utils.Parameters;

public class HICAPlanningAlgorithm extends BasePlanningAlgorithm {

    //-- ICA hyperparameters --
    private static final int NUM_COUNTRIES = Integer.getInteger("hica.pop", 30); 
    private static final int NUM_IMPERIALISTS = 6;        // ~20% of countries become imperialists
    private static final int MAX_ITER = 100;
    private static final double ASSIMILATION_PROB = 0.6;   // chance a gene moves toward the imperialist
    private static final double REVOLUTION_RATE = 0.1;     // chance a gene mutates randomly
    private static final double ZETA = 0.1;                // weight of mean colony cost in empire total cost

    //-- Budget and communication (aligned with OrderAwarePlanningBase) --
    private static final int EVAL_BUDGET = Integer.getInteger("evals", 3000);
    private static final boolean USE_COMM = Boolean.getBoolean("comm");
    private static final double INTER_DC_BW_MBPS = Double.parseDouble(System.getProperty("interDcBw", "15.0"));
    private static final double INTER_DC_LATENCY = Double.parseDouble(System.getProperty("interDcLatency", "0.05"));

    //-- Fitness weights (aligned with OrderAwarePlanningBase) --
    private static final double W1_MAKESPAN = Double.parseDouble(System.getProperty("w1", "1.0"));
    private static final double W2_COST = Double.parseDouble(System.getProperty("w2", "0.0"));
    private static final double W3_UTIL = Double.parseDouble(System.getProperty("w3", "0.0"));
    private static final double PRICE_BASE = 0.001;

    protected double w1Makespan = 1.0;
    protected double w2Cost = 0.0;
    protected double w3Util = 0.0;
    protected String normMode = "minmax";

    protected double minMakespanObserved = Double.POSITIVE_INFINITY;
    protected double maxMakespanObserved = Double.NEGATIVE_INFINITY;
    protected double minCostObserved = Double.POSITIVE_INFINITY;
    protected double maxCostObserved = Double.NEGATIVE_INFINITY;
    protected double minUtilObserved = Double.POSITIVE_INFINITY;
    protected double maxUtilObserved = Double.NEGATIVE_INFINITY;

    public static double bestEstimatedMakespan = 0.0;
    private int evalCount = 0;

    private boolean budgetLeft() {
        return evalCount < EVAL_BUDGET;
    }

    private double refMakespan = 1.0;
    private double refCost = 1.0;
    private double[][] interDcTransferDelay;
    private Map<Task, Integer> taskIndexMap;

    private List<Task> topoOrder;
    private static final long SEED = Long.getLong("seed", 42L);
    private final Random rnd = new Random(SEED);

    private static class Empire {
        int imperialistIdx;
        List<Integer> colonyIdx = new ArrayList<>();
    }

    @Override
    public void run() {
        topoOrder = topologicalSort();
        int n = topoOrder.size();
        int m = getVmList().size();

        taskIndexMap = new HashMap<>();
        for (int i = 0; i < n; i++) {
            taskIndexMap.put(topoOrder.get(i), i);
        }
        computeInterDcDelays();

        evalCount = 0;
        Log.printLine("HICA planning: " + n + " tasks, " + m + " VMs, budget " + EVAL_BUDGET + " evals, seed " + SEED);

        //-- 1. Initialize countries + reference normalization (Eq. 4) --
        int[][] countries = new int[NUM_COUNTRIES][n];
        double[] cost = new double[NUM_COUNTRIES];
        double sumMakespan = 0;
        double sumCost = 0;
        double[][] initMetrics = new double[NUM_COUNTRIES][3];

        for (int i = 0; i < NUM_COUNTRIES; i++) {
            for (int t = 0; t < n; t++) {
                countries[i][t] = rnd.nextInt(m);
            }
            repair(countries[i], m);
        }
        boolean hasW = System.getProperty("w1") != null || System.getProperty("w2") != null || System.getProperty("w3") != null;
        boolean mo = Boolean.getBoolean("mo");
        if (hasW) {
            w1Makespan = Double.parseDouble(System.getProperty("w1", "1.0"));
            w2Cost = Double.parseDouble(System.getProperty("w2", "0.0"));
            w3Util = Double.parseDouble(System.getProperty("w3", "0.0"));
        } else if (mo) {
            w1Makespan = 0.5;
            w2Cost = 0.3;
            w3Util = 0.2;
        } else {
            w1Makespan = 1.0;
            w2Cost = 0.0;
            w3Util = 0.0;
        }

        double sumW = w1Makespan + w2Cost + w3Util;
        if (sumW > 0) {
            w1Makespan /= sumW;
            w2Cost /= sumW;
            w3Util /= sumW;
        }
        // Default to "ref" (Paper Eq. 4) for consistent, stationary multi-objective scaling across all algorithms
        normMode = System.getProperty("norm", "ref");

        for (int i = 0; i < NUM_COUNTRIES; i++) {
            initMetrics[i] = computeRawMetrics(countries[i]);
            evalCount++;
            sumMakespan += initMetrics[i][0];
            sumCost += initMetrics[i][1];
            updateMetricBounds(initMetrics[i][0], initMetrics[i][1], initMetrics[i][2]);
        }
        refMakespan = sumMakespan / NUM_COUNTRIES;
        refCost = sumCost / NUM_COUNTRIES;

        for (int i = 0; i < NUM_COUNTRIES; i++) {
            cost[i] = calcFitness(initMetrics[i]);
        }

        //-- 2. Select imperialists --
        Integer[] order = sortIndicesByCost(cost);
        List<Empire> empires = new ArrayList<>();
        for (int k = 0; k < NUM_IMPERIALISTS; k++) {
            Empire e = new Empire();
            e.imperialistIdx = order[k];
            empires.add(e);
        }

        //-- 3. Distributing remaining countries as colonies --
        double[] impCost = new double[NUM_IMPERIALISTS];
        for (int k = 0; k < NUM_IMPERIALISTS; k++) {
            impCost[k] = cost[empires.get(k).imperialistIdx];
        }
        double maxImpCost = max(impCost);
        double[] power = new double[NUM_IMPERIALISTS];
        double powerSum = 0;
        for (int k = 0; k < NUM_IMPERIALISTS; k++) {
            power[k] = maxImpCost - impCost[k] + 1e-9; // lower cost --> higher power
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
        while (idx < remaining.size()) {       // leftover countries join the strongest empire
            empires.get(0).colonyIdx.add(remaining.get(idx));
            idx++;
        }

        //-- 4. Main ICA loop: assimilation, revolution, competition --
        int iter = 0;
        int maxIter = 10 * EVAL_BUDGET;
        while (budgetLeft() && iter < maxIter && empires.size() > 1) {

            for (Empire empire : empires) {
                if (!budgetLeft()) break;
                int[] imperialist = countries[empire.imperialistIdx];
                List<Integer> colonies = new ArrayList<>(empire.colonyIdx);

                for (int colIdx : colonies) {
                    if (!budgetLeft()) break;
                    assimilate(countries[colIdx], imperialist);
                    revolt(countries[colIdx], m);
                    repair(countries[colIdx], m); // repair after assimilation + revolution
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

            if (empires.size() > 1) {
                imperialisticCompetition(empires, cost);
            }

            if (iter % 10 == 0) {
                Empire strongest = strongestEmpire(empires, cost);
                double[] bMetrics = computeRawMetrics(countries[strongest.imperialistIdx]);
                Log.printLine("HICA step " + iter + " | evals " + evalCount
                        + " | best est. makespan " + String.format("%.3f", bMetrics[0])
                        + " | best cost " + String.format("%.4f", cost[strongest.imperialistIdx])
                        + " | empires " + empires.size());
            }
            iter++;
        }

        //-- 5. Apply the best mapping found --
        Empire best = strongestEmpire(empires, cost);
        int[] bestMapping = countries[best.imperialistIdx];
        double[] bestMetrics = computeRawMetrics(bestMapping);
        bestEstimatedMakespan = bestMetrics[0];

        for (int t = 0; t < n; t++) {
            Task task = topoOrder.get(t);
            CondorVM vm = (CondorVM) getVmList().get(bestMapping[t]);
            task.setVmId(vm.getId());
            task.setPriority(t);
        }
        Log.printLine("HICA planning finished after " + iter + " steps, " + evalCount + " evaluations"
                + "\n  -> estimated makespan: " + String.format("%.3f", bestMetrics[0])
                + " | estimated cost: " + String.format("%.4f", bestMetrics[1])
                + " | utilization: " + String.format("%.3f", bestMetrics[2])
                + "\n  -> compare the estimate with the simulated makespan printed by the driver");
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

    private void computeInterDcDelays() {
        if (!USE_COMM) return;
        int n = topoOrder.size();
        interDcTransferDelay = new double[n][n];
        double interDcBwMBps = INTER_DC_BW_MBPS; // inter-DC WAN bandwidth
        double minLatency = INTER_DC_LATENCY;    // baseline WAN latency

        for (int pIdx = 0; pIdx < n; pIdx++) {
            Task parent = topoOrder.get(pIdx);
            List<FileItem> parentFiles = parent.getFileList();
            for (Object cObj : parent.getChildList()) {
                Task child = (Task) cObj;
                Integer cIdx = taskIndexMap.get(child);
                if (cIdx == null) continue;
                List<FileItem> childFiles = child.getFileList();

                double bytes = 0.0;
                for (FileItem pf : parentFiles) {
                    if (pf.getType() != Parameters.FileType.OUTPUT) continue;
                    for (FileItem cf : childFiles) {
                        if (cf.getType() == Parameters.FileType.INPUT && cf.getName().equals(pf.getName())) {
                            bytes += cf.getSize();
                            break;
                        }
                    }
                }
                double mb = bytes / 1000000.0;
                interDcTransferDelay[pIdx][cIdx] = (mb / interDcBwMBps) + minLatency;
            }
        }
    }

    private double[] computeRawMetrics(int[] mapping) {
        int m = getVmList().size();
        int dc0VmCount = (m + 1) / 2;
        double[] vmFreeAt = new double[m];
        Map<Task, Double> finishTime = new HashMap<>();
        double totalCost = 0;
        double totalBusy = 0;

        for (int idx = 0; idx < topoOrder.size(); idx++) {
            Task task = topoOrder.get(idx);
            int vmIdx = mapping[idx];
            CondorVM vm = (CondorVM) getVmList().get(vmIdx);
            double mips = vm.getMips();

            double readyTime = vmFreeAt[vmIdx];
            boolean childInDC0 = vmIdx < dc0VmCount;

            for (Object p : task.getParentList()) {
                Task parent = (Task) p;
                Double pFinish = finishTime.get(parent);
                if (pFinish != null) {
                    int pIdx = taskIndexMap.get(parent);
                    boolean parentInDC0 = mapping[pIdx] < dc0VmCount;
                    double comm = (USE_COMM && interDcTransferDelay != null && parentInDC0 != childInDC0) ? interDcTransferDelay[pIdx][idx] : 0.0;
                    readyTime = Math.max(readyTime, pFinish + comm);
                }
            }

            double execTime = task.getCloudletLength() / mips;
            double finish = readyTime + execTime;

            finishTime.put(task, finish);
            vmFreeAt[vmIdx] = finish;

            double tier = mips / 1000.0;
            double vmPricePerSec = PRICE_BASE * (tier * tier);
            totalCost += execTime * vmPricePerSec;
            totalBusy += execTime;
        }

        double makespan = 0;
        for (double t : vmFreeAt) {
            makespan = Math.max(makespan, t);
        }

        double totalAvailable = makespan * m;
        double resourceUtil = totalAvailable > 0 ? totalBusy / totalAvailable : 0;

        return new double[]{makespan, totalCost, resourceUtil};
    }

    public void updateMetricBounds(double makespan, double cost, double util) {
        if (makespan < minMakespanObserved) minMakespanObserved = makespan;
        if (makespan > maxMakespanObserved) maxMakespanObserved = makespan;
        if (cost < minCostObserved) minCostObserved = cost;
        if (cost > maxCostObserved) maxCostObserved = cost;
        if (util < minUtilObserved) minUtilObserved = util;
        if (util > maxUtilObserved) maxUtilObserved = util;
    }

    public void setWeights(double w1, double w2, double w3) {
        this.w1Makespan = w1;
        this.w2Cost = w2;
        this.w3Util = w3;
        double sum = w1 + w2 + w3;
        if (sum > 0) {
            this.w1Makespan /= sum;
            this.w2Cost /= sum;
            this.w3Util /= sum;
        }
    }

    private double calcFitness(double[] metrics) {
        updateMetricBounds(metrics[0], metrics[1], metrics[2]);

        double normM;
        double normC;
        double normU;

        if ("ref".equalsIgnoreCase(normMode)) {
            normM = refMakespan > 0 ? metrics[0] / refMakespan : metrics[0];
            normC = refCost > 0 ? metrics[1] / refCost : metrics[1];
            normU = 1.0 - metrics[2];
        } else {
            normM = (maxMakespanObserved > minMakespanObserved + 1e-6)
                    ? (metrics[0] - minMakespanObserved) / (maxMakespanObserved - minMakespanObserved)
                    : (refMakespan > 0 ? metrics[0] / refMakespan : metrics[0]);
            normC = (maxCostObserved > minCostObserved + 1e-6)
                    ? (metrics[1] - minCostObserved) / (maxCostObserved - minCostObserved)
                    : (refCost > 0 ? metrics[1] / refCost : metrics[1]);
            normU = (maxUtilObserved > minUtilObserved + 1e-6)
                    ? (maxUtilObserved - metrics[2]) / (maxUtilObserved - minUtilObserved)
                    : (1.0 - metrics[2]);
        }

        return w1Makespan * normM + w2Cost * normC + w3Util * normU;
    }

    private double evaluate(int[] mapping) {
        evalCount++;
        return calcFitness(computeRawMetrics(mapping));
    }

    // ICA-specific operators
    // Assimilation
    private void assimilate(int[] colony, int[] imperialist) {
        for (int t = 0; t < colony.length; t++) {
            if (rnd.nextDouble() < ASSIMILATION_PROB) {
                colony[t] = imperialist[t];
            }
        }
    }

    // Revolution: random perturbation for exploration.
    // Applies one of:
    //   (a) swap two adjacent tasks in topological order (DAG-feasible)
    //   (b) reassign a task gene to a random VM
    private void revolt(int[] colony, int vmCount) {
        int n = colony.length;
        for (int t = 0; t < n; t++) {
            if (rnd.nextDouble() < REVOLUTION_RATE) {
                if (rnd.nextBoolean() && n > 1) {
                    // Option (a): adjacent task swap (DAG-safe)
                    int t2 = (t + 1 < n) ? t + 1 : t - 1;
                    if (!isDirectlyRelated(topoOrder.get(t), topoOrder.get(t2))) {
                        int tmp = colony[t];
                        colony[t] = colony[t2];
                        colony[t2] = tmp;
                    }
                } else {
                    // Option (b): random VM reassignment
                    colony[t] = rnd.nextInt(vmCount);
                }
            }
        }
    }

    // Total empire cost = imperialist cost + zeta * mean colony cost
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

    // Repair: clamp VM indices to valid range [0, vmCount).
    private void repair(int[] country, int vmCount) {
        for (int t = 0; t < country.length; t++) {
            if (country[t] < 0 || country[t] >= vmCount) {
                country[t] = rnd.nextInt(vmCount);
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