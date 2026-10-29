package org.workflowsim.planning;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.*;

import org.cloudbus.cloudsim.Log;
import org.workflowsim.CondorVM;
import org.workflowsim.Task;

/**
 * Shared base for population-based planners (CASSTO, GA, ...).
 *
 * Everything that must be IDENTICAL across algorithms for a fair comparison lives here:
 *   - the chromosome (Individual): VM per task + priority vector -> Kahn-decoded execution order
 *   - the decoder / repair (precedence feasibility is guaranteed by construction)
 *   - the evaluator (makespan, cost, utilization) and the fitness normalization
 *   - the evaluation budget counter, the seed, and the progress / trace logging
 *   - the order-swap move so that GA and CASSTO use the same rule
 *
 * Subclasses implement run(): call prepare() first, then search, then applySolution(best).
 *
 * JVM properties (all optional):
 *   -Dseed=N            random seed (default 42)
 *   -Devals=N           evaluation budget (default 3000)
 *   -Dw1= -Dw2= -Dw3=   fitness weights (default 1, 0, 0 = makespan only)
 *   -DsameVmSwap=true   order-swap partner is chosen from the same VM (default false = literal paper)
 *   -Dtrace=file.csv    append "seed,algo,step,evals,bestMakespan" rows for convergence plots
 */
public abstract class OrderAwarePlanningBase extends BasePlanningAlgorithm {

    // ---------- shared experiment settings ----------
    protected static final long SEED = Long.getLong("seed", 42L);
    protected static final int EVAL_BUDGET = Integer.getInteger("evals", 3000);
    protected static final boolean SAME_VM_SWAP = Boolean.getBoolean("sameVmSwap");

    protected static final double W1_MAKESPAN = dbl("w1", 1.0);
    protected static final double W2_COST = dbl("w2", 0.0);
    protected static final double W3_UTIL = dbl("w3", 0.0);

    // The paper does not define P_j. Design choice: price per second grows sub-linearly with speed,
    // so cost does NOT cancel out of (execTime * mips) like it did before.
    protected static final double PRICE_BASE = 0.001;
    protected static final double PRICE_EXPONENT = 0.7;

    protected static double dbl(String key, double def) {
        return Double.parseDouble(System.getProperty(key, Double.toString(def)));
    }

    // ---------- problem context (filled by prepare()) ----------
    protected List<Task> taskList;
    protected Map<Task, Integer> taskIndexMap;
    protected int n;                 // number of tasks
    protected int m;                 // number of VMs
    protected Random rnd;
    protected int evalCount;

    private double[] length;         // task length (MI)
    private double[] mips;           // VM speed
    private double[] price;          // VM price per second
    private int[][] parents;         // parents[i] = indices of parent tasks
    private double refMakespan = 1.0;
    private double refCost = 1.0;

    // ---------- chromosome ----------
    protected static class Individual {
        public int[] vm;             // task index -> VM index
        public int[] priority;       // task index -> priority (lower = earlier); rewritten to ranks by decode
        public List<Task> order;     // decoded, DAG-feasible execution order
        public double fitness;
        public double[] rawMetrics;  // [makespan, cost, utilization]
        public boolean dirty = true; // true = fitness is stale and evaluate() must recompute

        public Individual(int n) {
            vm = new int[n];
            priority = new int[n];
        }

        /** Exact copy, including evaluation state (dirty flag is copied too). */
        public Individual cloneIndividual() {
            Individual c = new Individual(vm.length);
            System.arraycopy(vm, 0, c.vm, 0, vm.length);
            System.arraycopy(priority, 0, c.priority, 0, priority.length);
            if (order != null) c.order = new ArrayList<>(order);
            c.fitness = fitness;
            if (rawMetrics != null) c.rawMetrics = rawMetrics.clone();
            c.dirty = dirty;
            return c;
        }
    }

    // =====================================================================
    // Setup
    // =====================================================================
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
            if (mips[j] > mips[fastest]) fastest = j;
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
                if (pi != null) s.add(pi);
            }
            parents[i] = new int[s.size()];
            int k = 0;
            for (int pi : s) parents[i][k++] = pi;
        }

        // Deterministic normalization: serial execution on the fastest VM.
        // Independent of seed and of the random initial population, so fitness is comparable across runs/algorithms.
        refMakespan = Math.max(1e-9, total / mips[fastest]);
        refCost = Math.max(1e-12, price[fastest] * refMakespan);
    }

    // =====================================================================
    // Decoder / repair (Algorithm 1 lines 4 & 24, Algorithm 2 lines 4 & 28)
    // =====================================================================

    /** Priority-guided Kahn. Any priority vector yields a precedence-feasible order. */
    protected List<Task> decodeOrder(final int[] priority) {
        int[] inDeg = new int[n];
        List<List<Integer>> children = new ArrayList<>(n);
        for (int i = 0; i < n; i++) children.add(new ArrayList<Integer>());
        for (int i = 0; i < n; i++) {
            inDeg[i] = parents[i].length;
            for (int p : parents[i]) children.get(p).add(i);
        }

        PriorityQueue<Integer> ready = new PriorityQueue<>(new Comparator<Integer>() {
            @Override
            public int compare(Integer a, Integer b) {
                int c = Integer.compare(priority[a], priority[b]);
                return c != 0 ? c : Integer.compare(a, b);   // deterministic tie-break
            }
        });
        for (int i = 0; i < n; i++) if (inDeg[i] == 0) ready.add(i);

        List<Task> order = new ArrayList<>(n);
        while (!ready.isEmpty()) {
            int u = ready.poll();
            order.add(taskList.get(u));
            for (int c : children.get(u)) {
                if (--inDeg[c] == 0) ready.add(c);
            }
        }
        if (order.size() != n) {
            throw new IllegalStateException("DAG decode produced " + order.size() + " of " + n
                    + " tasks (cycle or inconsistent parent/child lists)");
        }
        return order;
    }

    /** Re-decode the order from priorities and rewrite priorities as ranks (idempotent). */
    protected void redecode(Individual ind) {
        ind.order = decodeOrder(ind.priority);
        for (int rank = 0; rank < ind.order.size(); rank++) {
            ind.priority[taskIndexMap.get(ind.order.get(rank))] = rank;
        }
    }

    protected void repair(Individual ind) {
        for (int t = 0; t < n; t++) {
            if (ind.vm[t] < 0 || ind.vm[t] >= m) ind.vm[t] = rnd.nextInt(m);
        }
        redecode(ind);
    }

    /** Random VM assignment + random feasible topological order (Algorithm 1 L2 / Algorithm 2 L2). */
    protected Individual randomIndividual() {
        Individual ind = new Individual(n);
        List<Integer> perm = new ArrayList<>(n);
        for (int i = 0; i < n; i++) perm.add(i);
        Collections.shuffle(perm, rnd);
        for (int i = 0; i < n; i++) {
            ind.vm[i] = rnd.nextInt(m);
            ind.priority[i] = perm.get(i);
        }
        redecode(ind);
        ind.dirty = true;
        return ind;
    }

    // =====================================================================
    // Evaluation
    // =====================================================================

    /** Repairs if needed, scores, and counts toward the evaluation budget. No-op for clean individuals. */
    protected void evaluate(Individual ind) {
        if (!ind.dirty && ind.rawMetrics != null) return;
        repair(ind);
        ind.rawMetrics = computeRawMetrics(ind);
        ind.fitness = calcFitness(ind.rawMetrics);
        ind.dirty = false;
        evalCount++;
    }

    protected boolean budgetLeft() {
        return evalCount < EVAL_BUDGET;
    }

    /**
     * Hook for communication delay (Eq. 8, e.g. inter-datacenter transfer). Default: none.
     * Override in a subclass (or edit here) once you decide how to model transfers.
     */
    protected double commDelay(int parentTask, int childTask, int vmOfParent, int vmOfChild) {
        return 0.0;
    }

    /**
     * Strict per-VM queue in decoded order.
     * Returns [makespan, cost, utilization]. Cost = sum_j price_j * RT_j with RT_j = last finish time on VM j
     * (VM leased from t=0 until its last task completes; unused VMs cost nothing).
     */
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

    // =====================================================================
    // Shared move primitives
    // =====================================================================

    protected boolean isDirectlyRelated(Task a, Task b) {
        int ia = taskIndexMap.get(a), ib = taskIndexMap.get(b);
        for (int p : parents[ib]) if (p == ia) return true;
        for (int p : parents[ia]) if (p == ib) return true;
        return false;
    }

    /**
     * Order-swap move shared by GA (Algorithm 2 L23a) and CASSTO Brownian (Algorithm 1 L15a).
     * Default (literal paper): swap two ADJACENT tasks in the execution order if not parent/child.
     * With -DsameVmSwap=true: swap with the next task that runs on the SAME VM, because swapping
     * tasks on different VMs leaves the schedule unchanged under a per-VM queue evaluator.
     * Requires ind.order to be consistent with ind.priority (true after repair/redecode).
     */
    protected void orderSwapMove(Individual ind) {
        if (n < 2) return;
        int pos = rnd.nextInt(n - 1);
        Task a = ind.order.get(pos);
        int ia = taskIndexMap.get(a);

        if (SAME_VM_SWAP) {
            for (int q = pos + 1; q < n; q++) {
                Task b = ind.order.get(q);
                int ib = taskIndexMap.get(b);
                if (ind.vm[ib] == ind.vm[ia]) {
                    int tmp = ind.priority[ia]; ind.priority[ia] = ind.priority[ib]; ind.priority[ib] = tmp;
                    redecode(ind);          // non-adjacent swap may break precedence; Kahn fixes it
                    ind.dirty = true;
                    return;
                }
            }
            return;                          // no partner on this VM: no-op
        }

        Task b = ind.order.get(pos + 1);
        if (isDirectlyRelated(a, b)) return;
        int ib = taskIndexMap.get(b);
        int tmp = ind.priority[ia]; ind.priority[ia] = ind.priority[ib]; ind.priority[ib] = tmp;
        ind.order.set(pos, b);
        ind.order.set(pos + 1, a);
        ind.dirty = true;
    }

    // =====================================================================
    // Utilities
    // =====================================================================

    protected Individual bestOf(List<Individual> pop) {
        Individual best = pop.get(0);
        for (Individual x : pop) if (x.fitness < best.fitness) best = x;
        return best;
    }

    /** Schedule identity: execution order plus VM of each task. */
    protected String signature(Individual ind) {
        StringBuilder sb = new StringBuilder(n * 6);
        for (Task t : ind.order) {
            int i = taskIndexMap.get(t);
            sb.append(i).append(':').append(ind.vm[i]).append(',');
        }
        return sb.toString();
    }

    /** Write the best schedule back to the tasks and reorder the planner's task list in place. */
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
            for (int i = 0; i < best.order.size(); i++) raw.set(i, best.order.get(i));
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
        Log.printLine(algo + " planning finished after " + steps + " steps, " + evalCount + " evaluations"
                + "\n  -> estimated makespan: " + String.format("%.3f", best.rawMetrics[0])
                + " | estimated cost: " + String.format("%.4f", best.rawMetrics[1])
                + " | utilization: " + String.format("%.3f", best.rawMetrics[2])
                + "\n  -> compare the estimate with the simulated makespan printed by the driver");
        trace(algo, steps, best.rawMetrics[0]);
    }

    protected void trace(String algo, int step, double bestMakespan) {
        String path = System.getProperty("trace");
        if (path == null) return;
        try (PrintWriter pw = new PrintWriter(new FileWriter(path, true))) {
            pw.println(SEED + "," + algo + "," + step + "," + evalCount + "," + bestMakespan);
        } catch (IOException e) {
            // tracing is best-effort
        }
    }
}