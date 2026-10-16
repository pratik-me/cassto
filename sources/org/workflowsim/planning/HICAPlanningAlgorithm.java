package org.workflowsim.planning;

import java.util.*;

import org.cloudbus.cloudsim.Log;
import org.workflowsim.CondorVM;
import org.workflowsim.FileItem;
import org.workflowsim.Task;
import org.workflowsim.utils.Parameters;

public class HICAPlanningAlgorithm extends BasePlanningAlgorithm {

    //-- ICA hyperparameters --
    private static final int NUM_COUNTRIES = 30; 
    private static final int NUM_IMPERIALISTS = 6;        // ~20% of countries become imperialists
    private static final int MAX_ITER = 100;
    private static final double ASSIMILATION_PROB = 0.6;   // chance a gene moves toward the imperialist
    private static final double REVOLUTION_RATE = 0.1;     // chance a gene mutates randomly
    private static final double ZETA = 0.1;                // weight of mean colony cost in empire total cost

    //-- Fitness weights --
    private static final double W1_MAKESPAN = 0.5;
    private static final double W2_COST = 0.3;
    private static final double W3_UTIL = 0.2;
    private static final double PRICE_BASE = 0.01;

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

        Log.printLine("HICA planning: " + n + " tasks, " + m + " VMs.");

        //-- 1. Initialize countries + reference normalization (Eq. 4) --
        // HICA Hybrid: seed country[0] with a HEFT greedy schedule for warm start.
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
        for (int i = 0; i < NUM_COUNTRIES; i++) {
            initMetrics[i] = computeRawMetrics(countries[i]);
            sumMakespan += initMetrics[i][0];
            sumCost += initMetrics[i][1];
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
        for (int iter = 0; iter < MAX_ITER && empires.size() > 1; iter++) {

            for (Empire empire : empires) {
                int[] imperialist = countries[empire.imperialistIdx];
                List<Integer> colonies = new ArrayList<>(empire.colonyIdx);

                for (int colIdx : colonies) {
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

            imperialisticCompetition(empires, cost);

            if (iter % 10 == 0) {
                Empire strongest = strongestEmpire(empires, cost);
                Log.printLine("HICA iter " + iter + " best cost = " + cost[strongest.imperialistIdx]
                        + " empires remaining = " + empires.size());
            }
        }

        //-- 5. Apply the best mapping found --
        Empire best = strongestEmpire(empires, cost);
        int[] bestMapping = countries[best.imperialistIdx];
        for (int t = 0; t < n; t++) {
            Task task = topoOrder.get(t);
            CondorVM vm = (CondorVM) getVmList().get(bestMapping[t]);
            task.setVmId(vm.getId());
        }
        Log.printLine("HICA planning finished, best cost = " + cost[best.imperialistIdx]);
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
        int n = topoOrder.size();
        interDcTransferDelay = new double[n][n];
        double interDcBwMBps = 15.0; // 15 MB/s (~120 Mbps) inter-DC WAN bandwidth
        double minLatency = 0.05;    // 50ms baseline WAN latency

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
                    double comm = (parentInDC0 != childInDC0) ? interDcTransferDelay[pIdx][idx] : 0.0;
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

    private double calcFitness(double[] metrics) {
        double normMakespan = refMakespan > 0 ? metrics[0] / refMakespan : metrics[0];
        double normCost = refCost > 0 ? metrics[1] / refCost : metrics[1];
        double normUtil = 1.0 - metrics[2];

        return W1_MAKESPAN * normMakespan + W2_COST * normCost + W3_UTIL * normUtil;
    }

    private double evaluate(int[] mapping) {
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

    /**
     * HEFT-inspired greedy seed: assigns each task (in topological order) to the VM
     * that gives the earliest finish time, considering both VM availability and
     * parent-task completion times. This warm-starts the population with a strong
     * feasible schedule — the key "H" (hybrid) component of HICA.
     */
    private int[] heftSeed(int n, int m) {
        int[] assignment = new int[n];
        double[] vmFreeAt = new double[m];
        Map<Task, Double> finishTime = new HashMap<>();

        for (int idx = 0; idx < n; idx++) {
            Task task = topoOrder.get(idx);

            // Earliest time this task can start (all parents done)
            double dataReadyTime = 0;
            for (Object p : task.getParentList()) {
                dataReadyTime = Math.max(dataReadyTime, finishTime.get((Task) p));
            }

            // Pick the VM that gives the earliest finish time (EFT)
            int bestVm = 0;
            double bestEFT = Double.MAX_VALUE;
            for (int v = 0; v < m; v++) {
                CondorVM vm = (CondorVM) getVmList().get(v);
                double startTime = Math.max(vmFreeAt[v], dataReadyTime);
                double eft = startTime + task.getCloudletLength() / vm.getMips();
                if (eft < bestEFT) {
                    bestEFT = eft;
                    bestVm = v;
                }
            }

            assignment[idx] = bestVm;
            CondorVM chosenVm = (CondorVM) getVmList().get(bestVm);
            double startTime = Math.max(vmFreeAt[bestVm], dataReadyTime);
            double eft = startTime + task.getCloudletLength() / chosenVm.getMips();
            finishTime.put(task, eft);
            vmFreeAt[bestVm] = eft;
        }
        return assignment;
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