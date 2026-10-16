package org.workflowsim.planning;

import java.util.*;

import org.cloudbus.cloudsim.Log;
import org.workflowsim.CondorVM;
import org.workflowsim.FileItem;
import org.workflowsim.Task;
import org.workflowsim.utils.Parameters;

public class CASSTOPlanningAlgorithm extends BasePlanningAlgorithm {

    // =========================================================================
    // CASSTO Hyperparameters & Implementation Deviations from Paper
    // =========================================================================
    // 1. POP_SIZE = 30, MAX_ITER = 100, LEVY_LAMBDA = 1.5:
    //    Directly from paper Section 5.2.
    //
    // 2. Beta Parameter (Omitted / Implicit):
    //    Paper Eq. 5 specifies X^{t+1} = X^t + beta * N(0, 1) for continuous spaces.
    //    In discrete DAG scheduling, beta is omitted; local Brownian search is
    //    implemented via same-VM adjacent priority swaps and same-DC/adjacent-tier VM moves.
    //
    // 3. Alpha = 2.0 (Step Scale) & URGENT_BIAS = 0.7:
    //    Paper Eq. 5 includes alpha * Levy(lambda) but omits a numerical value for alpha.
    //    Alpha = 2.0 scales continuous Levy samples to a discrete number of task moves
    //    (yielding avg step ~ 3.1). Paper Sec. 4.3 mentions urgent task prioritization
    //    without an algorithmic mechanism; URGENT_BIAS = 0.7 biases 70% of Levy moves
    //    to reassign critical-path tasks.
    //
    // 4. Iteration Schedule (pBrownian: 0.2 -> 0.8):
    //    Paper Sec. 4.3 suggests switching based on diversity threshold delta = 0.25.
    //    Under greedy acceptance on discrete chromosomes, diversity collapses early.
    //    A linear exploration-to-exploitation ramp (0.2 -> 0.8) ensures global search
    //    early and fine-grained refinement late.
    //
    // 5. Fitness Weights (W1=0.5, W2=0.3, W3=0.2):
    //    Paper Eq. 4 leaves w1, w2, w3 as unspecified "user-defined" weights summing to 1.
    //    We assign W1=0.50 (Makespan), W2=0.30 (Cost), W3=0.20 (Resource Utilization)
    //    reflecting standard cloud workflow QoS priorities.
    // =========================================================================
    private static final int POP_SIZE = 30;
    private static final int MAX_ITER = 100;
    private static final double LEVY_LAMBDA = 1.5;
    private static final double DIVERSITY_THRESHOLD = 0.25;

    private static final double P_BROWNIAN_MIN = 0.2;
    private static final double P_BROWNIAN_MAX = 0.8;

    private static final double LEVY_ALPHA = 2.0;
    private static final double URGENT_BIAS = 0.7;

    private static final double W1_MAKESPAN = 0.5;
    private static final double W2_COST = 0.3;
    private static final double W3_UTIL = 0.2;
    private static final double PRICE_BASE = 0.01;


    private double refMakespan = 1.0;
    private double refCost = 1.0;
    public static double bestEstimatedMakespan = 0.0;

    private List<Task> taskList;
    private Map<Task, Integer> taskIndexMap;
    private Map<Task, Boolean> urgent;
    private int[] urgentIdx;
    private double[][] interDcTransferDelay;
    private List<Integer>[] vmNeighbors;

    private static final long SEED = Long.getLong("seed", 42L);
    private final Random rnd = new Random(SEED);

    private int brownianCount = 0;
    private int levyCount = 0;
    private int brownianWins = 0;
    private int levyWins = 0;
    private long totalLevySteps = 0;


    private static class Individual {
        int[] vm;          // task-to-VM assignment (task i -> VM index)
        int[] priority;    // permutation priority vector for Kahn's topological sort
        List<Task> order;  // decoded DAG-feasible execution order
        double fitness;
        double[] rawMetrics; // [makespan, cost, util]

        Individual(int n) {
            this.vm = new int[n];
            this.priority = new int[n];
        }

        Individual cloneIndividual() {
            Individual copy = new Individual(this.vm.length);
            System.arraycopy(this.vm, 0, copy.vm, 0, this.vm.length);
            System.arraycopy(this.priority, 0, copy.priority, 0, this.priority.length);
            if (this.order != null) {
                copy.order = new ArrayList<>(this.order);
            }
            copy.fitness = this.fitness;
            if (this.rawMetrics != null) {
                copy.rawMetrics = this.rawMetrics.clone();
            }
            return copy;
        }
    }

    @Override
    public void run() {
        taskList = new ArrayList<>();
        taskIndexMap = new HashMap<>();
        for (Object o : getTaskList()) {
            Task t = (Task) o;
            taskIndexMap.put(t, taskList.size());
            taskList.add(t);
        }

        int n = taskList.size();
        int m = getVmList().size();

        computeInterDcDelays();
        computeVmNeighbors(m);

        List<Task> baseTopoOrder = topologicalSort();
        urgent = computeUrgency(baseTopoOrder);

        List<Integer> u = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            if (urgent.get(taskList.get(i))) {
                u.add(i);
            }
        }
        urgentIdx = new int[u.size()];
        for (int i = 0; i < urgentIdx.length; i++) {
            urgentIdx[i] = u.get(i);
        }

        brownianCount = 0;
        levyCount = 0;
        brownianWins = 0;
        levyWins = 0;
        totalLevySteps = 0;

        Log.printLine("CASSTO planning: " + n + " tasks, " + m + " VMs, "
                + urgentIdx.length + " urgent tasks.");

        // -- 1. Initialization + reference normalization baselines (Algorithm 1: Lines 1-6) --
        Individual[] population = new Individual[POP_SIZE];
        boolean[] isRegenerated = new boolean[POP_SIZE];
        double sumMakespan = 0;
        double sumCost = 0;

        for (int i = 0; i < POP_SIZE; i++) {
            population[i] = new Individual(n);
            for (int t = 0; t < n; t++) {
                population[i].vm[t] = rnd.nextInt(m);
                population[i].priority[t] = rnd.nextInt(1000000);
            }

            repair(population[i], m);

            population[i].rawMetrics = computeRawMetrics(population[i]);
            sumMakespan += population[i].rawMetrics[0];
            sumCost += population[i].rawMetrics[1];
        }

        refMakespan = sumMakespan / POP_SIZE;
        refCost = sumCost / POP_SIZE;

        for (int i = 0; i < POP_SIZE; i++) {
            population[i].fitness = calcFitness(population[i].rawMetrics);
        }

        int bestIdx = bestOf(population);
        Individual best = population[bestIdx].cloneIndividual();
        double bestFitness = best.fitness;

        // -- 2. Main search loop (Algorithm 1: Lines 8-34) --
        for (int iter = 0; iter < MAX_ITER; iter++) {
            // Iteration method
            double pBrownian = P_BROWNIAN_MIN + (P_BROWNIAN_MAX - P_BROWNIAN_MIN) * iter / (double) MAX_ITER;

            for (int i = 0; i < POP_SIZE; i++) {
                boolean brownian = (rnd.nextDouble() < pBrownian);
                Individual candidate = population[i].cloneIndividual();

                if (brownian) {
                    brownianMove(candidate, m);
                } else {
                    levyMove(candidate, m);
                }

                // Algorithm Line 24: repair schedule S
                repair(candidate, m);

                candidate.rawMetrics = computeRawMetrics(candidate);
                candidate.fitness = calcFitness(candidate.rawMetrics);

                boolean wasRegen = isRegenerated[i];
                isRegenerated[i] = false;

                if (!wasRegen) {
                    if (brownian) brownianCount++;
                    else levyCount++;
                }

                if (candidate.fitness < population[i].fitness) {
                    if (!wasRegen) {
                        if (brownian) brownianWins++;
                        else levyWins++;
                    }
                    population[i] = candidate;
                }
                if (population[i].fitness < bestFitness) {
                    bestFitness = population[i].fitness;
                    best = population[i].cloneIndividual();
                }
            }

            // Sort population for diversity injection
            Integer[] order = sortIndices(population);

            // Diversity injection: regenerate worst individuals randomly
            int rCount = Math.max(1, (int) Math.round(0.067 * POP_SIZE));
            for (int r = 0; r < rCount; r++) {
                int worstIdx = order[POP_SIZE - 1 - r];
                for (int t = 0; t < n; t++) {
                    population[worstIdx].vm[t] = rnd.nextInt(m);
                    population[worstIdx].priority[t] = rnd.nextInt(1000000);
                }
                repair(population[worstIdx], m);
                population[worstIdx].rawMetrics = computeRawMetrics(population[worstIdx]);
                population[worstIdx].fitness = calcFitness(population[worstIdx].rawMetrics);

                if (population[worstIdx].fitness < bestFitness) {
                    bestFitness = population[worstIdx].fitness;
                    best = population[worstIdx].cloneIndividual();
                }
                isRegenerated[worstIdx] = true;
            }

            if (iter % 10 == 0) {
                double avgLevyStep = levyCount > 0 ? (double) totalLevySteps / levyCount : 0.0;
                double bWinRate = brownianCount > 0 ? 100.0 * brownianWins / brownianCount : 0.0;
                double lWinRate = levyCount > 0 ? 100.0 * levyWins / levyCount : 0.0;
                Log.printLine("CASSTO iter " + iter + " | best fit: " + String.format("%.4f", bestFitness)
                        + " | pBrownian: " + String.format("%.2f", pBrownian)
                        + " | Brownian moves: " + brownianCount + " (wins: " + brownianWins + ", " + String.format("%.1f%%", bWinRate) + ")"
                        + " | Levy moves: " + levyCount + " (wins: " + levyWins + ", " + String.format("%.1f%%", lWinRate) + ", avgStep: " + String.format("%.2f", avgLevyStep) + ")");
                Log.printLine("Best raw metrics: " + best.rawMetrics[0]);
            }
        }

        // -- 3. Apply the best mapping + execution order priority found --
        for (int rank = 0; rank < best.order.size(); rank++) {
            Task task = best.order.get(rank);
            int tIdx = taskIndexMap.get(task);
            CondorVM vm = (CondorVM) getVmList().get(best.vm[tIdx]);
            task.setVmId(vm.getId());
            task.setPriority(rank);
        }

        // Re-order the planner's task list to match the optimal topological execution order
        try {
            getTaskList().clear();
            getTaskList().addAll(best.order);
        } catch (Exception e) {
            // fallback if unmodifiable
        }

        double avgLevyStep = levyCount > 0 ? (double) totalLevySteps / levyCount : 0.0;
        double bWinRate = brownianCount > 0 ? 100.0 * brownianWins / brownianCount : 0.0;
        double lWinRate = levyCount > 0 ? 100.0 * levyWins / levyCount : 0.0;
        Log.printLine("CASSTO planning finished, best fitness = " + String.format("%.4f", bestFitness)
                + "\n  -> Brownian moves (local/nearby VM): " + brownianCount + " (improvements: " + brownianWins + ", winRate: " + String.format("%.1f%%", bWinRate) + ")"
                + "\n  -> Levy moves (global/any VM):       " + levyCount + " (improvements: " + levyWins + ", winRate: " + String.format("%.1f%%", lWinRate) + ", avgStep: " + String.format("%.2f", avgLevyStep) + ")");
        bestEstimatedMakespan = best.rawMetrics[0];
        Log.printLine("CASSTO Estimated Planner Makespan: " + best.rawMetrics[0]);
        double[] wcMetrics = computeRawMetricsWorkConserving(best);
        Log.printLine("CASSTO Work-Conserving Evaluator Makespan: " + wcMetrics[0]);
    }


    // Priority-Guided Kahn's Algorithm :
    private List<Task> decodeTopologicalOrder(int[] priority) {
        int n = taskList.size();
        int[] inDegree = new int[n];
        for (int i = 0; i < n; i++) {
            Task t = taskList.get(i);
            inDegree[i] = t.getParentList().size();
        }

        // PriorityQueue for ready tasks (inDegree == 0).
        PriorityQueue<Integer> ready = new PriorityQueue<>(new Comparator<Integer>() {
            @Override
            public int compare(Integer a, Integer b) {
                return Integer.compare(priority[a], priority[b]);
            }
        });

        for (int i = 0; i < n; i++) {
            if (inDegree[i] == 0) {
                ready.add(i);
            }
        }

        List<Task> order = new ArrayList<>(n);
        while (!ready.isEmpty()) {
            int uIdx = ready.poll();
            Task u = taskList.get(uIdx);
            order.add(u);

            for (Object childObj : u.getChildList()) {
                Task child = (Task) childObj;
                Integer cIdx = taskIndexMap.get(child);
                if (cIdx != null) {
                    inDegree[cIdx]--;
                    if (inDegree[cIdx] == 0) {
                        ready.add(cIdx);
                    }
                }
            }
        }
        return order;
    }

    private void repair(Individual ind, int vmCount) {
        for (int t = 0; t < ind.vm.length; t++) {
            if (ind.vm[t] < 0 || ind.vm[t] >= vmCount) {
                ind.vm[t] = rnd.nextInt(vmCount);
            }
        }
        // (i) Enforce precedence constraints via Kahn's priority decoder
        ind.order = decodeTopologicalOrder(ind.priority);
        for (int rank = 0; rank < ind.order.size(); rank++) {
            int tIdx = taskIndexMap.get(ind.order.get(rank));
            ind.priority[tIdx] = rank;
        }
    }

    private void computeInterDcDelays() {
        int n = taskList.size();
        interDcTransferDelay = new double[n][n];
        double interDcBwMBps = 15.0; // 15 MB/s (~120 Mbps) inter-DC WAN bandwidth
        double minLatency = 0.05;    // 50ms baseline WAN latency

        for (int pIdx = 0; pIdx < n; pIdx++) {
            Task parent = taskList.get(pIdx);
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
                // Eq. 8: Comm_i,j = D_i,j / BW_inter
                interDcTransferDelay[pIdx][cIdx] = (mb / interDcBwMBps) + minLatency;
            }
        }
    }

    private void computeVmNeighbors(int m) {
        int dc0VmCount = (m + 1) / 2;
        vmNeighbors = new List[m];
        for (int v = 0; v < m; v++) {
            vmNeighbors[v] = new ArrayList<>();
            boolean inDC0 = (v < dc0VmCount);
            int vTier = v % 3;
            for (int u = 0; u < m; u++) {
                if (u == v) continue;
                boolean uInDC0 = (u < dc0VmCount);
                if (uInDC0 == inDC0) {
                    int uTier = u % 3;
                    if (Math.abs(uTier - vTier) <= 1) {
                        vmNeighbors[v].add(u);
                    }
                }
            }
            if (vmNeighbors[v].isEmpty()) {
                vmNeighbors[v].add(v);
            }
        }
    }

    private void brownianMove(Individual ind, int vmCount) {
        int n = taskList.size();
        int moves = Math.max(1, Math.min(5, n / 25)); // small Omega set

        for (int k = 0; k < moves; k++) {
            if (rnd.nextBoolean() && n > 1) {
                // (a) Order move: pick two tasks on the SAME VM and swap their priorities (Kahn repairs any DAG violation)
                int pos = rnd.nextInt(n);
                Task a = ind.order.get(pos);
                int idxA = taskIndexMap.get(a);
                int vm = ind.vm[idxA];
                for (int q = pos + 1; q < n; q++) {
                    Task b = ind.order.get(q);
                    int idxB = taskIndexMap.get(b);
                    if (ind.vm[idxB] == vm) {
                        int tmpP = ind.priority[idxA];
                        ind.priority[idxA] = ind.priority[idxB];
                        ind.priority[idxB] = tmpP;
                        break;
                    }
                }
            } else {
                // (b) Nearby VM move: same datacenter and same or adjacent speed tier (true refinement)
                int tIdx = rnd.nextInt(n);
                int curVm = ind.vm[tIdx];
                List<Integer> neighbors = vmNeighbors[curVm];
                ind.vm[tIdx] = neighbors.get(rnd.nextInt(neighbors.size()));
            }
        }
    }

    private void levyMove(Individual ind, int vmCount) {
        int n = taskList.size();
        double step = LEVY_ALPHA * Math.abs(levySample(LEVY_LAMBDA));
        int moves = Math.min(n, Math.max(1, (int) Math.ceil(step)));
        totalLevySteps += moves;

        if (rnd.nextBoolean()) {
            // (a) Global VM jump: explore any VM across datacenters
            for (int k = 0; k < moves; k++) {
                int idx;
                if (urgentIdx.length > 0 && rnd.nextDouble() < URGENT_BIAS) {
                    idx = urgentIdx[rnd.nextInt(urgentIdx.length)];
                } else {
                    idx = rnd.nextInt(n);
                }
                ind.vm[idx] = rnd.nextInt(vmCount);
            }
        } else {
            // (b) Order move: shuffle priorities among tasks that share a VM
            List<Integer>[] tasksByVm = new List[vmCount];
            for (int v = 0; v < vmCount; v++) {
                tasksByVm[v] = new ArrayList<>();
            }
            for (int t = 0; t < n; t++) {
                tasksByVm[ind.vm[t]].add(t);
            }

            List<Integer> vmsWithMultiTasks = new ArrayList<>();
            for (int v = 0; v < vmCount; v++) {
                if (tasksByVm[v].size() > 1) {
                    vmsWithMultiTasks.add(v);
                }
            }

            if (!vmsWithMultiTasks.isEmpty()) {
                int chosenVm = vmsWithMultiTasks.get(rnd.nextInt(vmsWithMultiTasks.size()));
                List<Integer> vmTasks = tasksByVm[chosenVm];
                Collections.shuffle(vmTasks, rnd);
                int subsetSize = Math.min(moves, vmTasks.size());
                List<Integer> sub = vmTasks.subList(0, subsetSize);

                List<Integer> pVals = new ArrayList<>();
                for (int t : sub) {
                    pVals.add(ind.priority[t]);
                }
                Collections.shuffle(pVals, rnd);
                for (int i = 0; i < sub.size(); i++) {
                    ind.priority[sub.get(i)] = pVals.get(i);
                }
            } else {
                int idx = rnd.nextInt(n);
                ind.vm[idx] = rnd.nextInt(vmCount);
            }
        }
    }


    private List<Task> topologicalSort() {
        List<Task> order = new ArrayList<>();
        Set<Task> visited = new HashSet<>();
        for (Task t : taskList) {
            visitTask(t, visited, order);
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

    // Urgency indicator U_i (Paper Eq. 6 & Critical Path Membership)
    // Forward pass: tl[i] = longest path from workflow entry to the start of task i
    // Backward pass: bl[i] = longest path from task i to workflow exit (including task i)
    // Total path through task i: pathLength(i) = tl[i] + bl[i]
    // Task i is urgent if pathLength(i) >= (1 - 0.05) * criticalPathLength (i.e. on or within 5% of critical path)
    private Map<Task, Boolean> computeUrgency(List<Task> order) {
        int n = order.size();
        Map<Task, Double> tl = new HashMap<>();
        Map<Task, Double> bl = new HashMap<>();

        // 1. Forward pass: compute top-level tl[i] in topological order
        for (Task t : order) {
            double maxParentFinish = 0.0;
            for (Object pObj : t.getParentList()) {
                Task p = (Task) pObj;
                Double pStart = tl.get(p);
                double pFinish = (pStart != null ? pStart : 0.0) + p.getCloudletLength();
                maxParentFinish = Math.max(maxParentFinish, pFinish);
            }
            tl.put(t, maxParentFinish);
        }

        // 2. Backward pass: compute bottom-level bl[i] in reverse topological order
        for (int i = n - 1; i >= 0; i--) {
            Task t = order.get(i);
            double maxChildPath = 0.0;
            for (Object cObj : t.getChildList()) {
                Task c = (Task) cObj;
                Double cBl = bl.get(c);
                if (cBl != null) {
                    maxChildPath = Math.max(maxChildPath, cBl);
                }
            }
            bl.put(t, t.getCloudletLength() + maxChildPath);
        }

        // 3. Compute global critical path length across all tasks
        double criticalPathLength = 0.0;
        for (Task t : order) {
            double totalPath = tl.get(t) + bl.get(t);
            criticalPathLength = Math.max(criticalPathLength, totalPath);
        }

        // 4. Mark tasks as urgent if their path is within 5% of the critical path length
        Map<Task, Boolean> urgencyMap = new HashMap<>();
        double threshold = (1.0 - 0.05) * criticalPathLength; // (1 - 0.05) * CPL
        for (Task t : order) {
            double totalPath = tl.get(t) + bl.get(t);
            urgencyMap.put(t, totalPath >= threshold);
        }
        return urgencyMap;
    }

    // Fitness evaluation (Normalized QoS objective, paper Eq. 4 / Eq. 12)
    private double[] computeRawMetrics(Individual ind) {
        return computeRawMetricsWorkConserving(ind);
    }

    // Work-conserving fitness evaluation matching WorkflowSim's runtime dispatch semantics
    // with inter-datacenter transfer latency (Eq. 8) and tiered VM pricing
    private double[] computeRawMetricsWorkConserving(Individual ind) {
        int n = taskList.size();
        int m = getVmList().size();
        int dc0VmCount = (m + 1) / 2;

        int[] remainingParents = new int[n];
        for (int i = 0; i < n; i++) {
            remainingParents[i] = taskList.get(i).getParentList().size();
        }

        List<PriorityQueue<Integer>> vmReadyQueue = new ArrayList<>(m);
        for (int v = 0; v < m; v++) {
            vmReadyQueue.add(new PriorityQueue<>(new Comparator<Integer>() {
                @Override
                public int compare(Integer a, Integer b) {
                    return Integer.compare(ind.priority[a], ind.priority[b]);
                }
            }));
        }

        double[] dataReadyAt = new double[n];

        for (int i = 0; i < n; i++) {
            if (remainingParents[i] == 0) {
                vmReadyQueue.get(ind.vm[i]).add(i);
            }
        }

        // Event queue: [timestamp, eventType, taskIndex, vmIndex]
        // eventType: 1.0 = TASK_FINISH, 2.0 = DATA_READY
        PriorityQueue<double[]> events = new PriorityQueue<>(new Comparator<double[]>() {
            @Override
            public int compare(double[] a, double[] b) {
                int cmp = Double.compare(a[0], b[0]);
                if (cmp != 0) return cmp;
                return Double.compare(b[1], a[1]); // DATA_READY (2.0) before TASK_FINISH (1.0) if same time
            }
        });

        boolean[] vmBusy = new boolean[m];
        double totalCost = 0;
        double totalBusy = 0;

        for (int v = 0; v < m; v++) {
            if (!vmReadyQueue.get(v).isEmpty()) {
                int t = vmReadyQueue.get(v).poll();
                vmBusy[v] = true;
                CondorVM vm = (CondorVM) getVmList().get(v);
                double execTime = taskList.get(t).getCloudletLength() / vm.getMips();
                events.add(new double[] { execTime, 1.0, t, v });
                double tier = vm.getMips() / 1000.0;
                double vmPricePerSec = PRICE_BASE * (tier * tier);
                totalCost += execTime * vmPricePerSec;
                totalBusy += execTime;
            }
        }

        double currentTime = 0;
        while (!events.isEmpty()) {
            double[] event = events.poll();
            currentTime = event[0];
            double eventType = event[1];
            int taskIdx = (int) event[2];
            int vmId = (int) event[3];

            if (eventType == 1.0) { // TASK_FINISH
                vmBusy[vmId] = false;
                Task tObj = taskList.get(taskIdx);

                for (Object cObj : tObj.getChildList()) {
                    Task child = (Task) cObj;
                    int cIdx = taskIndexMap.get(child);
                    boolean diffDC = (ind.vm[taskIdx] < dc0VmCount) != (ind.vm[cIdx] < dc0VmCount);
                    double comm = diffDC ? interDcTransferDelay[taskIdx][cIdx] : 0.0;
                    dataReadyAt[cIdx] = Math.max(dataReadyAt[cIdx], currentTime + comm);

                    remainingParents[cIdx]--;
                    if (remainingParents[cIdx] == 0) {
                        int cVm = ind.vm[cIdx];
                        if (dataReadyAt[cIdx] <= currentTime) {
                            vmReadyQueue.get(cVm).add(cIdx);
                            if (!vmBusy[cVm]) {
                                int nextT = vmReadyQueue.get(cVm).poll();
                                vmBusy[cVm] = true;
                                CondorVM vm = (CondorVM) getVmList().get(cVm);
                                double exec = taskList.get(nextT).getCloudletLength() / vm.getMips();
                                events.add(new double[] { currentTime + exec, 1.0, nextT, cVm });
                                double tier = vm.getMips() / 1000.0;
                                double vmPricePerSec = PRICE_BASE * (tier * tier);
                                totalCost += exec * vmPricePerSec;
                                totalBusy += exec;
                            }
                        } else {
                            // Inter-DC transfer in flight
                            events.add(new double[] { dataReadyAt[cIdx], 2.0, cIdx, cVm });
                        }
                    }
                }

                // VM is now free, check if there are pending ready tasks on this VM
                if (!vmBusy[vmId] && !vmReadyQueue.get(vmId).isEmpty()) {
                    int nextT = vmReadyQueue.get(vmId).poll();
                    vmBusy[vmId] = true;
                    CondorVM vm = (CondorVM) getVmList().get(vmId);
                    double exec = taskList.get(nextT).getCloudletLength() / vm.getMips();
                    events.add(new double[] { currentTime + exec, 1.0, nextT, vmId });
                    double tier = vm.getMips() / 1000.0;
                    double vmPricePerSec = PRICE_BASE * (tier * tier);
                    totalCost += exec * vmPricePerSec;
                    totalBusy += exec;
                }
            } else if (eventType == 2.0) { // DATA_READY
                int cVm = vmId;
                vmReadyQueue.get(cVm).add(taskIdx);
                if (!vmBusy[cVm]) {
                    int nextT = vmReadyQueue.get(cVm).poll();
                    vmBusy[cVm] = true;
                    CondorVM vm = (CondorVM) getVmList().get(cVm);
                    double exec = taskList.get(nextT).getCloudletLength() / vm.getMips();
                    events.add(new double[] { currentTime + exec, 1.0, nextT, cVm });
                    double tier = vm.getMips() / 1000.0;
                    double vmPricePerSec = PRICE_BASE * (tier * tier);
                    totalCost += exec * vmPricePerSec;
                    totalBusy += exec;
                }
            }
        }

        double makespan = currentTime;
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

    // Diversity computation
    private double populationDiversity(Individual[] population) {
        int popSize = population.length;
        int n = taskList.size();
        double totalDist = 0;
        int pairs = 0;

        for (int i = 0; i < popSize; i++) {
            for (int j = i + 1; j < popSize; j++) {
                int diff = 0;
                for (int t = 0; t < n; t++) {
                    if (population[i].vm[t] != population[j].vm[t]) {
                        diff++;
                    }
                }
                totalDist += (double) diff / n;
                pairs++;
            }
        }
        return pairs > 0 ? totalDist / pairs : 0;
    }

    private boolean isDirectlyRelated(Task a, Task b) {
        for (Object p : b.getParentList()) {
            if (p == a) return true;
        }
        for (Object p : a.getParentList()) {
            if (p == b) return true;
        }
        return false;
    }

    /** Mantegna's algorithm for a symmetric Levy-stable step. */
    private double levySample(double lambda) {
        double sigma = Math.pow(
                (gamma(1 + lambda) * Math.sin(Math.PI * lambda / 2))
                        / (gamma((1 + lambda) / 2) * lambda * Math.pow(2, (lambda - 1) / 2)),
                1 / lambda);
        double u = rnd.nextGaussian() * sigma;
        double v = rnd.nextGaussian();
        return u / Math.pow(Math.abs(v), 1 / lambda);
    }

    /** Lanczos approximation of the Gamma function. */
    private double gamma(double x) {
        double[] g = { 676.5203681218851, -1259.1392167224028, 771.32342877765313,
                -176.61502916214059, 12.507343278686905, -0.13857109526572012,
                9.9843695780195716e-6, 1.5056327351493116e-7 };
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

    private int bestOf(Individual[] pop) {
        int best = 0;
        for (int i = 1; i < pop.length; i++) {
            if (pop[i].fitness < pop[best].fitness) {
                best = i;
            }
        }
        return best;
    }

    private Integer[] sortIndices(Individual[] pop) {
        Integer[] idx = new Integer[pop.length];
        for (int i = 0; i < idx.length; i++) {
            idx[i] = i;
        }
        Arrays.sort(idx, Comparator.comparingDouble(i -> pop[i].fitness));
        return idx;
    }
}