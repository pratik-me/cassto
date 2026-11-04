package org.workflowsim.planning;

import java.util.*;

import org.cloudbus.cloudsim.Log;
import org.workflowsim.Task;

public class CASSTOPaperPlanningAlgorithm extends OrderAwarePlanningBase {

    private static final int POP_SIZE = Integer.getInteger("cassto.pop", 30);
    private static final double DELTA = dbl("cassto.delta", 0.25);
    private static final double LAMBDA = Math.min(1.9, Math.max(0.3, dbl("cassto.lambda", 1.5)));
    private static final double ALPHA = dbl("cassto.alpha", 2.0);
    private static final int OMEGA_B_PROP = Integer.getInteger("cassto.paper.omegaBrownian", -1);
    private static final double OMEGA_CAP_FRAC = dbl("cassto.paper.omegaCap", 1.0);
    private static final double NEIGHBOR_RATIO = dbl("cassto.paper.neighborRatio", Double.POSITIVE_INFINITY);
    private static final boolean VM_LOCAL_SWAP = Boolean.parseBoolean(System.getProperty("cassto.paper.vmLocalSwap", "false"));
    private static final boolean SKIP_NOOP = Boolean.parseBoolean(System.getProperty("cassto.paper.skipNoop", "false"));
    private static final boolean REL_DIVERSITY = Boolean.parseBoolean(System.getProperty("cassto.paper.relDiversity", "false"));
    private static final boolean INVERT = Boolean.parseBoolean(System.getProperty("cassto.paper.invert", "false"));
    private static final int ELITE = Integer.getInteger("cassto.paper.elite", 1);
    private static final int REGEN = Integer.getInteger("cassto.paper.regen", 0);
    private static final int STALL_LIMIT = Integer.getInteger("cassto.stallLimit", 200);

    private static final double SIGMA = computeSigma(LAMBDA);
    private static final double INV_LAMBDA = 1.0 / LAMBDA;

    public static double bestEstimatedMakespan = 0.0;

    private int[][] children;
    private List<Integer>[] vmNeighbors;
    private int omegaB;
    private int omegaCap;

    // statistics
    private int brownianIters, levyIters;
    private int brownianAttempts, levyAttempts, brownianWins, levyWins;
    private long levyOmegaSum;

    @SuppressWarnings("unchecked")
    @Override
    public void run() {
        prepare();
        bestEstimatedMakespan = 0.0;
        brownianIters = levyIters = 0;
        brownianAttempts = levyAttempts = brownianWins = levyWins = 0;
        levyOmegaSum = 0;

        buildChildren();
        buildNeighbors();
        omegaB = OMEGA_B_PROP > 0 ? Math.min(n, OMEGA_B_PROP) : Math.max(1, (int) Math.round(0.03 * n));
        omegaCap = Math.min(n, Math.max(omegaB + 1, (int) Math.round(OMEGA_CAP_FRAC * n)));

        Log.printLine("CASSTO-PAPER planning: " + n + " tasks, " + m + " VMs, pop " + POP_SIZE
                + ", delta " + DELTA + (REL_DIVERSITY ? " (relative D)" : " (absolute D)")
                + ", lambda " + LAMBDA + ", alpha " + ALPHA + ", |Omega_B| " + omegaB
                + ", |Omega_L| cap " + omegaCap + ", budget " + EVAL_BUDGET + " evals, seed " + SEED
                + (INVERT ? ", INVERTED phase rule" : "")
                + (VM_LOCAL_SWAP ? ", vmLocalSwap" : "") + (SKIP_NOOP ? ", skipNoop" : ""));

        // ---- Lines 2-6: initialise, repair, evaluate ----
        Individual[] pop = new Individual[POP_SIZE];
        for (int i = 0; i < POP_SIZE; i++) {
            pop[i] = randomIndividual();
            repair(pop[i]);
            pop[i].rawMetrics = computeRawMetrics(pop[i]);
            evalCount++;
            pop[i].dirty = false;
        }
        refreshBounds(pop, null);
        double initD = Math.max(1e-9, diversity(pop));

        // ---- Line 7 ----
        Individual best = bestOf(Arrays.asList(pop)).cloneIndividual();

        // ---- Lines 8-34 ----
        int iter = 0, stall = 0;
        int maxIter = 10 * EVAL_BUDGET;
        while (budgetLeft() && iter < maxIter) {
            refreshBounds(pop, best); // no-op for norm=ref; keeps min-max scoring consistent otherwise

            // Line 9: diversity once per iteration
            double D = diversity(pop);
            double dCmp = REL_DIVERSITY ? D / initD : D;
            // Line 12: D < delta -> Brownian
            boolean brownian = INVERT ? (dCmp >= DELTA) : (dCmp < DELTA);
            if (brownian) brownianIters++; else levyIters++;

            int before = evalCount;
            for (int i = 0; i < POP_SIZE; i++) {
                if (!budgetLeft()) break;

                Individual cand = pop[i].cloneIndividual(); // line 11: S <- S_i
                if (brownian) brownianPhase(cand); else levyPhase(cand);

                if (SKIP_NOOP && cand.dirty && signature(cand).equals(signature(pop[i]))) {
                    cand.dirty = false; // identical schedule: nothing to evaluate
                }
                boolean attempted = cand.dirty;
                evaluate(cand); // lines 24-25: repair + F(S)

                if (attempted) {
                    if (brownian) brownianAttempts++; else levyAttempts++;
                }
                if (cand.fitness < pop[i].fitness) { // lines 26-28 (strict)
                    if (attempted) {
                        if (brownian) brownianWins++; else levyWins++;
                    }
                    pop[i] = cand;
                }
                if (pop[i].fitness < best.fitness) { // lines 29-31
                    best = pop[i].cloneIndividual();
                }
            }

            // Line 33: optional elitism / regeneration of the worst r
            if (REGEN > 0) {
                Integer[] order = sortIndices(pop);
                int r = Math.min(REGEN, Math.max(0, POP_SIZE - ELITE));
                for (int k = 0; k < r && budgetLeft(); k++) {
                    int w = order[POP_SIZE - 1 - k];
                    pop[w] = randomIndividual();
                    evaluate(pop[w]);
                    if (pop[w].fitness < best.fitness) best = pop[w].cloneIndividual();
                }
            }

            stall = (evalCount == before) ? stall + 1 : 0;
            if (stall >= STALL_LIMIT) {
                Log.printLine("CASSTO-PAPER stopped early: no fresh evaluations for " + STALL_LIMIT + " iterations.");
                break;
            }
            if (iter % 10 == 0) {
                logProgress("CASSTO-PAPER", iter, best);
                Log.printLine("CASSTO-PAPER iter " + iter + " | D " + String.format("%.4f", D)
                        + " | phase " + (brownian ? "Brownian" : "Levy")
                        + " | B-iters " + brownianIters + " L-iters " + levyIters);
            }
            iter++;
        }

        // Line 35
        applySolution(best);
        bestEstimatedMakespan = best.rawMetrics[0];

        double bRate = brownianAttempts > 0 ? 100.0 * brownianWins / brownianAttempts : 0.0;
        double lRate = levyAttempts > 0 ? 100.0 * levyWins / levyAttempts : 0.0;
        double avgOmega = levyAttempts > 0 ? (double) levyOmegaSum / Math.max(1, levyIters * POP_SIZE) : 0.0;
        Log.printLine("  -> Final diversity D: " + String.format("%.4f", diversity(pop)) + " (delta " + DELTA + ")"
                + "\n  -> Brownian phase iterations: " + brownianIters + " | attempts " + brownianAttempts
                + " | wins " + brownianWins + " (" + String.format("%.1f%%", bRate) + ")"
                + "\n  -> Levy phase iterations:     " + levyIters + " | attempts " + levyAttempts
                + " | wins " + levyWins + " (" + String.format("%.1f%%", lRate) + ")"
                + " | avg |Omega| " + String.format("%.2f", avgOmega));
        logFinal("CASSTO-PAPER", best, iter);
    }

    // ------------------------------------------------------------------ phases

    /** Lines 13-16. */
    private void brownianPhase(Individual ind) {
        for (int t : sampleTasks(omegaB)) {
            if (rnd.nextBoolean() && n > 1) {
                adjacentSwap(ind, t);        // (a)
            } else {
                nearbyVmMove(ind, t);        // (b)
            }
        }
    }

    /** Lines 18-22. */
    private void levyPhase(Individual ind) {
        double L = Math.abs(levySample());
        if (Double.isNaN(L)) L = 1.0;
        int size = (int) Math.min((double) n, Math.ceil(ALPHA * L)); // |Omega| ~ alpha * L
        size = Math.max(omegaB + 1, size);                           // "a larger set of tasks"
        size = Math.min(omegaCap, size);
        levyOmegaSum += size;
        for (int t : sampleTasks(size)) {
            if (rnd.nextBoolean()) {
                ind.vm[t] = differentVm(ind.vm[t]);                  // (a) any VM, across datacenters
                ind.dirty = true;
            } else {
                relocateInOrder(ind, t);                             // (b) larger dependency-preserving permutation
            }
        }
    }

    // ------------------------------------------------------------------ moves

    /** Brownian (a): swap the task with an adjacent one in the topological order. */
    private void adjacentSwap(Individual ind, int t) {
        List<Task> order = ind.order;
        Task task = taskList.get(t);
        int pos = order.indexOf(task);

        if (VM_LOCAL_SWAP) {
            // adjacent in the task's own VM queue (non-adjacent in the global order); Kahn decode repairs precedence
            int partner = -1;
            boolean fwd = rnd.nextBoolean();
            for (int pass = 0; pass < 2 && partner < 0; pass++) {
                boolean dir = (pass == 0) == fwd;
                if (dir) {
                    for (int q = pos + 1; q < n; q++) if (ind.vm[taskIndexMap.get(order.get(q))] == ind.vm[t]) { partner = q; break; }
                } else {
                    for (int q = pos - 1; q >= 0; q--) if (ind.vm[taskIndexMap.get(order.get(q))] == ind.vm[t]) { partner = q; break; }
                }
            }
            if (partner < 0) return;
            int ib = taskIndexMap.get(order.get(partner));
            List<Task> snapshot = new ArrayList<>(order);
            int tmp = ind.priority[t];
            ind.priority[t] = ind.priority[ib];
            ind.priority[ib] = tmp;
            redecode(ind);
            if (!ind.order.equals(snapshot)) ind.dirty = true;
            return;
        }

        int q = rnd.nextBoolean() ? pos + 1 : pos - 1;
        if (q < 0) q = pos + 1;
        if (q >= n) q = pos - 1;
        if (q < 0 || q >= n) return;
        Task other = order.get(q);
        if (isDirectlyRelated(task, other)) return; // swap would violate precedence
        int ib = taskIndexMap.get(other);
        int tmp = ind.priority[t];
        ind.priority[t] = ind.priority[ib];
        ind.priority[ib] = tmp;
        order.set(pos, other);
        order.set(q, task);
        ind.dirty = true;
    }

    /** Brownian (b): move task to a nearby VM (same datacenter, optional MIPS-ratio bound). */
    private void nearbyVmMove(Individual ind, int t) {
        List<Integer> nb = vmNeighbors[ind.vm[t]];
        int nv = nb.get(rnd.nextInt(nb.size()));
        if (nv != ind.vm[t]) {
            ind.vm[t] = nv;
            ind.dirty = true;
        }
    }

    /**
     * Levy (b): "larger permutation within a valid topological order". Interpreted as relocating the
     * task to a uniformly random position inside its feasible window (after its last parent, before
     * its first child), which can move it arbitrarily far while preserving every precedence edge.
     */
    private void relocateInOrder(Individual ind, int t) {
        Task task = taskList.get(t);
        int cur = ind.order.indexOf(task);
        List<Task> tmp = new ArrayList<>(ind.order);
        tmp.remove(cur);

        boolean[] isPar = new boolean[n];
        boolean[] isChild = new boolean[n];
        for (int p : parents[t]) isPar[p] = true;
        for (int c : children[t]) isChild[c] = true;

        int lo = 0, hi = tmp.size();
        for (int k = 0; k < tmp.size(); k++) {
            int id = taskIndexMap.get(tmp.get(k));
            if (isPar[id]) lo = k + 1;
            if (isChild[id] && k < hi) hi = k;
        }
        if (hi < lo) return; // cannot happen for a valid order; defensive
        int idx = lo + rnd.nextInt(hi - lo + 1);
        if (idx == cur) return;

        tmp.add(idx, task);
        ind.order = tmp;
        for (int r = 0; r < n; r++) ind.priority[taskIndexMap.get(tmp.get(r))] = r;
        ind.dirty = true;
    }

    // ------------------------------------------------------------------ helpers

    private int[] sampleTasks(int k) {
        k = Math.max(1, Math.min(k, n));
        int[] a = new int[n];
        for (int i = 0; i < n; i++) a[i] = i;
        for (int i = 0; i < k; i++) {
            int j = i + rnd.nextInt(n - i);
            int tmp = a[i]; a[i] = a[j]; a[j] = tmp;
        }
        return Arrays.copyOf(a, k);
    }

    private void buildChildren() {
        List<Integer>[] tmp = new List[n];
        for (int i = 0; i < n; i++) tmp[i] = new ArrayList<>();
        for (int c = 0; c < n; c++) for (int p : parents[c]) tmp[p].add(c);
        children = new int[n][];
        for (int i = 0; i < n; i++) {
            children[i] = new int[tmp[i].size()];
            for (int k = 0; k < children[i].length; k++) children[i][k] = tmp[i].get(k);
        }
    }

    @SuppressWarnings("unchecked")
    private void buildNeighbors() {
        vmNeighbors = new List[m];
        for (int v = 0; v < m; v++) {
            vmNeighbors[v] = new ArrayList<>();
            for (int u = 0; u < m; u++) {
                if (u == v || dcOfVm(u) != dcOfVm(v)) continue;
                double ratio = Math.max(vmMips(u), vmMips(v)) / Math.min(vmMips(u), vmMips(v));
                if (ratio <= NEIGHBOR_RATIO) vmNeighbors[v].add(u);
            }
            if (vmNeighbors[v].isEmpty()) vmNeighbors[v].add(v);
        }
    }

    /** Mean pairwise Hamming distance of task->VM assignments, normalised by n (so D is in [0,1)). */
    private double diversity(Individual[] pop) {
        double total = 0.0;
        int pairs = 0;
        for (int i = 0; i < pop.length; i++) {
            for (int j = i + 1; j < pop.length; j++) {
                int diff = 0;
                for (int t = 0; t < n; t++) if (pop[i].vm[t] != pop[j].vm[t]) diff++;
                total += (double) diff / n;
                pairs++;
            }
        }
        return pairs > 0 ? total / pairs : 0.0;
    }

    private Integer[] sortIndices(Individual[] pop) {
        Integer[] idx = new Integer[pop.length];
        for (int i = 0; i < idx.length; i++) idx[i] = i;
        Arrays.sort(idx, Comparator.comparingDouble(i -> pop[i].fitness));
        return idx;
    }

    // ------------------------------------------------------------------ Levy sampling (Mantegna)

    private static double computeSigma(double lambda) {
        return Math.pow(
                (gamma(1 + lambda) * Math.sin(Math.PI * lambda / 2.0))
                        / (gamma((1 + lambda) / 2.0) * lambda * Math.pow(2.0, (lambda - 1.0) / 2.0)),
                1.0 / lambda);
    }

    private double levySample() {
        double u = rnd.nextGaussian() * SIGMA;
        double v = rnd.nextGaussian();
        return u / Math.pow(Math.abs(v), INV_LAMBDA);
    }

    private static double gamma(double x) {
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
}