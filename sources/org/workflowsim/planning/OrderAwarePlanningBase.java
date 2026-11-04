package org.workflowsim.planning;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.*;

import org.cloudbus.cloudsim.Log;
import org.workflowsim.CondorVM;
import org.workflowsim.FileItem;
import org.workflowsim.utils.Parameters;
import org.workflowsim.Task;

public abstract class OrderAwarePlanningBase extends BasePlanningAlgorithm {

    // ---------- shared experiment settings ----------
    protected static final long SEED = Long.getLong("seed", 42L);
    protected static final int EVAL_BUDGET = Integer.getInteger("evals", 3000);
    protected static final boolean SAME_VM_SWAP = Boolean.getBoolean("sameVmSwap");

    // Legacy static constants (retained for backward compatibility)
    protected static final double W1_MAKESPAN = dbl("w1", 1.0);
    protected static final double W2_COST = dbl("w2", 0.0);
    protected static final double W3_UTIL = dbl("w3", 0.0);

    // Multi-objective normalized weights & normalization mode (Paper Eq. 4 & Eq. 12)
    protected double w1Makespan = 1.0;
    protected double w2Cost = 0.0;
    protected double w3Util = 0.0;
    protected String normMode = "ref";

    // Dynamic metric bounds for Min-Max normalization (Paper Eq. 12, Page 16)
    protected double minMakespanObserved = Double.POSITIVE_INFINITY;
    protected double maxMakespanObserved = Double.NEGATIVE_INFINITY;
    protected double minCostObserved = Double.POSITIVE_INFINITY;
    protected double maxCostObserved = Double.NEGATIVE_INFINITY;
    protected double minUtilObserved = Double.POSITIVE_INFINITY;
    protected double maxUtilObserved = Double.NEGATIVE_INFINITY;

    // Frozen bounds per iteration (ensures consistent scoring across candidate evaluations)
    protected double frozenMinMakespan = Double.POSITIVE_INFINITY;
    protected double frozenMaxMakespan = Double.NEGATIVE_INFINITY;
    protected double frozenMinCost = Double.POSITIVE_INFINITY;
    protected double frozenMaxCost = Double.NEGATIVE_INFINITY;
    protected double frozenMinUtil = Double.POSITIVE_INFINITY;
    protected double frozenMaxUtil = Double.NEGATIVE_INFINITY;

    protected static final boolean USE_COMM = Boolean.getBoolean("comm");
    protected static final double INTER_DC_BW_MBPS = dbl("interDcBw", 15.0);
    protected static final double INTER_DC_LATENCY = dbl("interDcLatency", 0.05);
    private final Map<Long, Double> edgeDelay = new HashMap<>();

    // The paper does not define P_j. Design choice: price per second grows
    // sub-linearly with speed, so cost does NOT cancel out of (execTime * mips)
    // like it did before.
    protected static final double PRICE_BASE = 0.001;
    protected static final double PRICE_EXPONENT = 0.7;

    protected static double dbl(String key, double def) {
        return Double.parseDouble(System.getProperty(key, Double.toString(def)));
    }

    protected int dcOfVm(int v) {
        return v < (m + 1) / 2 ? 0 : 1;
    }

    protected double vmMips(int v) {
        return mips[v];
    }

    protected List<Task> taskList;
    protected Map<Task, Integer> taskIndexMap;
    protected int n; // number of tasks
    protected int m; // number of VMs
    protected Random rnd;
    protected int evalCount;

    protected double[] length; // task length (MI)
    protected double[] mips; // VM speed
    protected double[] price; // VM price \ sec
    protected int[][] parents; // parents[i] = indices of parent tasks
    protected double refMakespan = 1.0;
    protected double refCost = 1.0;

    protected static class Individual {
        public int[] vm; // task index -> VM index
        public int[] priority; // task index -> priority (lower = earlier)
        public List<Task> order; // decoded, DAG-feasible execution order
        public double fitness;
        public double[] rawMetrics;
        public boolean dirty = true;

        public Individual(int n) {
            vm = new int[n];
            priority = new int[n];
        }

        public Individual cloneIndividual() {
            Individual c = new Individual(vm.length);
            System.arraycopy(vm, 0, c.vm, 0, vm.length);
            System.arraycopy(priority, 0, c.priority, 0, priority.length);
            if (order != null)
                c.order = new ArrayList<>(order);
            c.fitness = fitness;
            if (rawMetrics != null)
                c.rawMetrics = rawMetrics.clone();
            c.dirty = dirty;
            return c;
        }
    }

    private void buildEdgeDelays() {
        edgeDelay.clear();
        if (!USE_COMM)
            return;
        for (int c = 0; c < n; c++) {
            Task child = taskList.get(c);
            for (int p : parents[c]) {
                double bytes = 0;
                for (FileItem pf : taskList.get(p).getFileList()) {
                    if (pf.getType() != Parameters.FileType.OUTPUT)
                        continue;
                    for (FileItem cf : child.getFileList()) {
                        if (cf.getType() == Parameters.FileType.INPUT && cf.getName().equals(pf.getName())) {
                            bytes += cf.getSize();
                            break;
                        }
                    }
                }
                if (bytes > 0)
                    edgeDelay.put((long) p * n + c, bytes / 1e6 / INTER_DC_BW_MBPS + INTER_DC_LATENCY);
            }
        }
    }

    @SuppressWarnings("rawtypes")
    protected void prepare() {
        rnd = new Random(SEED);
        evalCount = 0;

        taskList = new ArrayList<>();
        taskIndexMap = new HashMap<>();
        for (Object o : (List) getTaskList()) {
            Task t = (Task) o;
            taskIndexMap.put(t, taskList.size());
            taskList.add(t);
        }
        n = taskList.size();
        m = getVmList().size();

        mips = new double[m];
        price = new double[m];
        int fastest = 0;
        for (int j = 0; j < m; j++) {
            CondorVM vm = (CondorVM) getVmList().get(j);
            mips[j] = vm.getMips();
            price[j] = PRICE_BASE * Math.pow(mips[j] / 1000.0, PRICE_EXPONENT);
            if (mips[j] > mips[fastest])
                fastest = j;
        }

        length = new double[n];
        double total = 0;
        for (int i = 0; i < n; i++) {
            length[i] = taskList.get(i).getCloudletLength();
            total += length[i];
        }

        parents = new int[n][];
        for (int i = 0; i < n; i++) {
            LinkedHashSet<Integer> s = new LinkedHashSet<>();
            for (Object p : taskList.get(i).getParentList()) {
                Integer pi = taskIndexMap.get((Task) p);
                if (pi != null)
                    s.add(pi);
            }
            parents[i] = new int[s.size()];
            int k = 0;
            for (int pi : s)
                parents[i][k++] = pi;
        }

        double sumPrice = 0.0;
        for (int j = 0; j < m; j++) {
            sumPrice += price[j];
        }

        // Realistic parallel reference scale (across m VMs)
        refMakespan = Math.max(1e-9, total / (mips[fastest] * Math.max(1.0, m / 2.0)));
        refCost = Math.max(1e-12, sumPrice * refMakespan);

        // Reset dynamic bounds
        minMakespanObserved = Double.POSITIVE_INFINITY;
        maxMakespanObserved = Double.NEGATIVE_INFINITY;
        minCostObserved = Double.POSITIVE_INFINITY;
        maxCostObserved = Double.NEGATIVE_INFINITY;
        minUtilObserved = Double.POSITIVE_INFINITY;
        maxUtilObserved = Double.NEGATIVE_INFINITY;

        frozenMinMakespan = Double.POSITIVE_INFINITY;
        frozenMaxMakespan = Double.NEGATIVE_INFINITY;
        frozenMinCost = Double.POSITIVE_INFINITY;
        frozenMaxCost = Double.NEGATIVE_INFINITY;
        frozenMinUtil = Double.POSITIVE_INFINITY;
        frozenMaxUtil = Double.NEGATIVE_INFINITY;

        // Multi-objective weights configuration and normalization (w1 + w2 + w3 = 1.0)
        boolean hasW = System.getProperty("w1") != null || System.getProperty("w2") != null || System.getProperty("w3") != null;
        boolean mo = Boolean.getBoolean("mo");
        if (hasW) {
            w1Makespan = dbl("w1", 1.0);
            w2Cost = dbl("w2", 0.0);
            w3Util = dbl("w3", 0.0);
        } else if (mo) {
            // Multi-objective defaults (Section 4.2 / Section 5.6)
            w1Makespan = 0.5;
            w2Cost = 0.3;
            w3Util = 0.2;
        } else {
            // Default single-objective replication mode
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

        normMode = System.getProperty("norm", "ref");
        buildEdgeDelays();
    }

    protected List<Task> decodeOrder(final int[] priority) {
        int[] inDeg = new int[n];
        List<List<Integer>> children = new ArrayList<>(n);
        for (int i = 0; i < n; i++)
            children.add(new ArrayList<Integer>());
        for (int i = 0; i < n; i++) {
            inDeg[i] = parents[i].length;
            for (int p : parents[i])
                children.get(p).add(i);
        }

        PriorityQueue<Integer> ready = new PriorityQueue<>(new Comparator<Integer>() {
            @Override
            public int compare(Integer a, Integer b) {
                int c = Integer.compare(priority[a], priority[b]);
                return c != 0 ? c : Integer.compare(a, b); // deterministic tie-break
            }
        });
        for (int i = 0; i < n; i++)
            if (inDeg[i] == 0)
                ready.add(i);

        List<Task> order = new ArrayList<>(n);
        while (!ready.isEmpty()) {
            int u = ready.poll();
            order.add(taskList.get(u));
            for (int c : children.get(u)) {
                if (--inDeg[c] == 0)
                    ready.add(c);
            }
        }
        if (order.size() != n) {
            throw new IllegalStateException("DAG decode produced " + order.size() + " of " + n
                    + " tasks (cycle or inconsistent parent/child lists)");
        }
        return order;
    }

    // Re-decode order from priorities and rewrite priorities as ranks (idempotent).
    protected void redecode(Individual ind) {
        ind.order = decodeOrder(ind.priority);
        for (int rank = 0; rank < ind.order.size(); rank++) {
            ind.priority[taskIndexMap.get(ind.order.get(rank))] = rank;
        }
    }

    protected void repair(Individual ind) {
        for (int t = 0; t < n; t++) {
            if (ind.vm[t] < 0 || ind.vm[t] >= m)
                ind.vm[t] = rnd.nextInt(m);
        }
        redecode(ind);
    }

    // Random VM assignment + random feasible topo. order
    protected Individual randomIndividual() {
        Individual ind = new Individual(n);
        List<Integer> perm = new ArrayList<>(n);
        for (int i = 0; i < n; i++)
            perm.add(i);
        Collections.shuffle(perm, rnd);
        for (int i = 0; i < n; i++) {
            ind.vm[i] = rnd.nextInt(m);
            ind.priority[i] = perm.get(i);
        }
        redecode(ind);
        ind.dirty = true;
        return ind;
    }

    protected boolean evaluate(Individual ind) {
        if (!ind.dirty && ind.rawMetrics != null)
            return false;
        repair(ind);
        double[] mt = computeRawMetrics(ind);
        evalCount++;
        ind.rawMetrics = mt.clone();
        ind.fitness = calcFitness(mt);
        ind.dirty = false;
        return true;
    }

    protected boolean budgetLeft() {
        return evalCount < EVAL_BUDGET;
    }

    // Hook for communication delay (Eq. 8: inter-datacenter transfer).
    protected double commDelay(int p, int c, int vp, int vc) {
        // Default none.
        if (!USE_COMM || dcOfVm(vp) == dcOfVm(vc))
            return 0.0;
        Double d = edgeDelay.get((long) p * n + c);
        return d == null ? 0.0 : d;
    }

    protected double[] computeRawMetrics(Individual ind) {
        double[] vmFree = new double[m];
        double[] finish = new double[n];
        double busy = 0;

        for (Task task : ind.order) {
            int i = taskIndexMap.get(task);
            int v = ind.vm[i];
            double ready = vmFree[v];
            for (int p : parents[i]) {
                ready = Math.max(ready, finish[p] + commDelay(p, i, ind.vm[p], v));
            }
            double exec = length[i] / mips[v];
            finish[i] = ready + exec;
            vmFree[v] = finish[i];
            busy += exec;
        }

        double makespan = 0, cost = 0;
        for (int j = 0; j < m; j++) {
            makespan = Math.max(makespan, vmFree[j]);
            cost += price[j] * vmFree[j];
        }
        double util = makespan > 0 ? busy / (makespan * m) : 0;
        return new double[] { makespan, cost, util };
    }

    public void updateMetricBounds(double makespan, double cost, double util) {
        if (makespan < minMakespanObserved) minMakespanObserved = makespan;
        if (makespan > maxMakespanObserved) maxMakespanObserved = makespan;
        if (cost < minCostObserved) minCostObserved = cost;
        if (cost > maxCostObserved) maxCostObserved = cost;
        if (util < minUtilObserved) minUtilObserved = util;
        if (util > maxUtilObserved) maxUtilObserved = util;
    }

    public void refreshBounds(Individual[] pop, Individual best) {
        if (pop == null || pop.length == 0) return;
        double minM = Double.POSITIVE_INFINITY, maxM = Double.NEGATIVE_INFINITY;
        double minC = Double.POSITIVE_INFINITY, maxC = Double.NEGATIVE_INFINITY;
        double minU = Double.POSITIVE_INFINITY, maxU = Double.NEGATIVE_INFINITY;

        List<Individual> all = new ArrayList<>(pop.length + (best != null ? 1 : 0));
        for (Individual p : pop) {
            if (p != null && p.rawMetrics != null) all.add(p);
        }
        if (best != null && best.rawMetrics != null) {
            all.add(best);
        }

        for (Individual ind : all) {
            double m = ind.rawMetrics[0];
            double c = ind.rawMetrics[1];
            double u = ind.rawMetrics[2];

            if (m < minM) minM = m;
            if (m > maxM) maxM = m;
            if (c < minC) minC = c;
            if (c > maxC) maxC = c;
            if (u < minU) minU = u;
            if (u > maxU) maxU = u;
        }

        if (minM < Double.POSITIVE_INFINITY) {
            frozenMinMakespan = minM;
            frozenMaxMakespan = maxM;
            frozenMinCost = minC;
            frozenMaxCost = maxC;
            frozenMinUtil = minU;
            frozenMaxUtil = maxU;

            updateMetricBounds(minM, minC, minU);
            updateMetricBounds(maxM, maxC, maxU);
        }

        // Re-score every individual under the refreshed bounds
        for (Individual p : pop) {
            if (p != null && p.rawMetrics != null) {
                p.fitness = calcFitness(p.rawMetrics);
            }
        }
        if (best != null && best.rawMetrics != null) {
            best.fitness = calcFitness(best.rawMetrics);
        }
    }

    public void refreshBounds(Collection<Individual> pop, Individual best) {
        if (pop != null && !pop.isEmpty()) {
            refreshBounds(pop.toArray(new Individual[0]), best);
        }
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

    protected double calcFitness(double[] mt) {
        double normM;
        double normC;
        double normU;

        if ("ref".equalsIgnoreCase(normMode)) {
            // Eq. 4: Reference normalization
            normM = mt[0] / refMakespan;
            normC = mt[1] / refCost;
            normU = 1.0 - mt[2];
        } else {
            // Eq. 12: Min-Max normalization using frozen population bounds
            double curMinM = (frozenMinMakespan < Double.POSITIVE_INFINITY) ? frozenMinMakespan : minMakespanObserved;
            double curMaxM = (frozenMaxMakespan > Double.NEGATIVE_INFINITY) ? frozenMaxMakespan : maxMakespanObserved;
            double curMinC = (frozenMinCost < Double.POSITIVE_INFINITY) ? frozenMinCost : minCostObserved;
            double curMaxC = (frozenMaxCost > Double.NEGATIVE_INFINITY) ? frozenMaxCost : maxCostObserved;
            double curMinU = (frozenMinUtil < Double.POSITIVE_INFINITY) ? frozenMinUtil : minUtilObserved;
            double curMaxU = (frozenMaxUtil > Double.NEGATIVE_INFINITY) ? frozenMaxUtil : maxUtilObserved;

            double spanM = curMaxM - curMinM;
            double spanC = curMaxC - curMinC;
            double spanU = curMaxU - curMinU;

            normM = (spanM > 1e-6) ? (mt[0] - curMinM) / spanM : (mt[0] / refMakespan);
            normC = (spanC > 1e-6) ? (mt[1] - curMinC) / spanC : (mt[1] / refCost);
            normU = (spanU > 1e-6) ? (curMaxU - mt[2]) / spanU : (1.0 - mt[2]);
        }

        return w1Makespan * normM + w2Cost * normC + w3Util * normU;
    }

    protected boolean isDirectlyRelated(Task a, Task b) {
        int ia = taskIndexMap.get(a), ib = taskIndexMap.get(b);
        for (int p : parents[ib])
            if (p == ia)
                return true;
        for (int p : parents[ia])
            if (p == ib)
                return true;
        return false;
    }

    protected void orderSwapMove(Individual ind) {
        if (n < 2)
            return;
        int pos = rnd.nextInt(n - 1);
        Task a = ind.order.get(pos);
        int ia = taskIndexMap.get(a);

        if (SAME_VM_SWAP) {
            for (int q = pos + 1; q < n; q++) {
                Task b = ind.order.get(q);
                int ib = taskIndexMap.get(b);
                if (ind.vm[ib] == ind.vm[ia]) {
                    int tmp = ind.priority[ia];
                    ind.priority[ia] = ind.priority[ib];
                    ind.priority[ib] = tmp;
                    redecode(ind); // non-adjacent swap may break precedence; Kahn fixes it
                    ind.dirty = true;
                    return;
                }
            }
            return; // no partner on this VM: no-op
        }

        Task b = ind.order.get(pos + 1);
        if (isDirectlyRelated(a, b))
            return;
        int ib = taskIndexMap.get(b);
        int tmp = ind.priority[ia];
        ind.priority[ia] = ind.priority[ib];
        ind.priority[ib] = tmp;
        ind.order.set(pos, b);
        ind.order.set(pos + 1, a);
        ind.dirty = true;
    }

    protected int differentVm(int currentVm) {
        if (m < 2)
            return currentVm;
        int nv = rnd.nextInt(m - 1);
        return nv >= currentVm ? nv + 1 : nv;
    }

    protected void reassignToDifferentVm(Individual ind, int task) {
        if (m < 2)
            return;
        ind.vm[task] = differentVm(ind.vm[task]);
        ind.dirty = true;
    }

    protected Individual bestOf(List<Individual> pop) {
        Individual best = pop.get(0);
        for (Individual x : pop)
            if (x.fitness < best.fitness)
                best = x;
        return best;
    }

    protected String signature(Individual ind) {
        StringBuilder[] per = new StringBuilder[m];
        for (Task t : ind.order) {
            int i = taskIndexMap.get(t), v = ind.vm[i];
            if (per[v] == null)
                per[v] = new StringBuilder().append(v).append(':');
            per[v].append(i).append(',');
        }
        StringBuilder out = new StringBuilder(n * 6);
        for (int v = 0; v < m; v++)
            if (per[v] != null)
                out.append(per[v]).append(';');
        return out.toString();
    }

    @SuppressWarnings({ "rawtypes", "unchecked" })
    protected void applySolution(Individual best) {
        for (int rank = 0; rank < best.order.size(); rank++) {
            Task task = best.order.get(rank);
            CondorVM vm = (CondorVM) getVmList().get(best.vm[taskIndexMap.get(task)]);
            task.setVmId(vm.getId());
            task.setPriority(rank);
        }
        try {
            List raw = (List) getTaskList();
            for (int i = 0; i < best.order.size(); i++)
                raw.set(i, best.order.get(i));
        } catch (UnsupportedOperationException e) {
            Log.printLine("WARNING: task list is read-only; execution order is conveyed only via Task.priority");
        }
    }

    protected void logProgress(String algo, int step, Individual best) {
        Log.printLine(algo + " step " + step + " | evals " + evalCount
                + " | best est. makespan " + String.format("%.3f", best.rawMetrics[0])
                + " | best fitness " + String.format("%.4f", best.fitness));
        trace(algo, step, best.rawMetrics[0]);
    }

    protected void logFinal(String algo, Individual best, int steps) {
        Log.printLine(algo + " planning finished after " + steps + " steps, " + evalCount + " evaluations "
                + "\n  -> estimated makespan: " + String.format("%.3f", best.rawMetrics[0])
                + " | estimated cost: " + String.format("%.4f", best.rawMetrics[1])
                + " | utilization: " + String.format("%.3f", best.rawMetrics[2])
                + "\n  -> compare the estimate with the simulated makespan printed by the driver");
        trace(algo, steps, best.rawMetrics[0]);
    }

    protected void trace(String algo, int step, double bestMakespan) {
        String path = System.getProperty("trace");
        if (path == null)
            return;
        try (PrintWriter pw = new PrintWriter(new FileWriter(path, true))) {
            pw.println(SEED + "," + algo + "," + step + "," + evalCount + "," + bestMakespan);
        } catch (IOException e) {
            // tracing is best-effort
        }
    }
}