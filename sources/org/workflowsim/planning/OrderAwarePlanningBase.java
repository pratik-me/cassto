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

    protected static final double W1_MAKESPAN = dbl("w1", 1.0);
    protected static final double W2_COST = dbl("w2", 0.0);
    protected static final double W3_UTIL = dbl("w3", 0.0);

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

    // ---------- problem context (filled by prepare()) ----------
    protected List<Task> taskList;
    protected Map<Task, Integer> taskIndexMap;
    protected int n; // number of tasks
    protected int m; // number of VMs
    protected Random rnd;
    protected int evalCount;

    private double[] length; // task length (MI)
    private double[] mips; // VM speed
    private double[] price; // VM price per second
    private int[][] parents; // parents[i] = indices of parent tasks
    private final Map<String, double[]> cache = new HashMap<>(); // clear in prepare()
    protected int cacheHits;
    private double refMakespan = 1.0;
    private double refCost = 1.0;

    // ---------- chromosome ----------
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

        /** Exact copy, including evaluation state (dirty flag is copied too). */
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
        cache.clear();
        cacheHits = 0;

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

        refMakespan = Math.max(1e-9, total / mips[fastest]);
        refCost = Math.max(1e-12, price[fastest] * refMakespan);
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

    /**
     * Re-decode the order from priorities and rewrite priorities as ranks
     * (idempotent).
     */
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

    // Random VM assignment + random feasible topological order
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
        String key = signature(ind);
        double[] mt = cache.get(key);
        boolean fresh = (mt == null);
        if (fresh) {
            mt = computeRawMetrics(ind);
            cache.put(key, mt);
        } else {
            cacheHits++;
        }
        evalCount++;
        ind.rawMetrics = mt.clone();
        ind.fitness = calcFitness(mt);
        ind.dirty = false;
        return fresh;
    }

    protected boolean budgetLeft() {
        return evalCount < EVAL_BUDGET;
    }

    // Hook for communication delay (Eq. 8, e.g. inter-datacenter transfer).
    // Default: none.
    protected double commDelay(int p, int c, int vp, int vc) {
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

    /** Eq. 4 with seed-independent reference values. Lower is better. */
    protected double calcFitness(double[] mt) {
        return W1_MAKESPAN * (mt[0] / refMakespan)
                + W2_COST * (mt[1] / refCost)
                + W3_UTIL * (1.0 - mt[2]);
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

    /** Schedule identity: execution order plus VM of each task. */
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

    /**
     * Write the best schedule back to the tasks and reorder the planner's task list
     * in place.
     */
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
        Log.printLine(algo + " planning finished after " + steps + " steps, " + evalCount + " evaluations (cache hits: "
                + cacheHits + ")"
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