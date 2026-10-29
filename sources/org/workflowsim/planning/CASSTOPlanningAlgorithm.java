package org.workflowsim.planning;

import java.util.*;

import org.cloudbus.cloudsim.Log;
import org.workflowsim.Task;

public class CASSTOPlanningAlgorithm extends OrderAwarePlanningBase {
    private static final int POP_SIZE = 30;
    private static final double LEVY_LAMBDA = 1.5;
    private static final double SIGMA_LEVY = computeSigma(LEVY_LAMBDA);
    private static final double INV_LEVY_LAMBDA = 1.0 / LEVY_LAMBDA;

    private static final double P_BROWNIAN_MIN = 0.2;
    private static final double P_BROWNIAN_MAX = 0.8;

    private static final double LEVY_ALPHA = 2.0;
    private static final double URGENT_BIAS = 0.7;

    private static final boolean DEDUPE = Boolean.parseBoolean(System.getProperty("cassto.dedupe", "true"));
    private static final int STALL_LIMIT = Integer.getInteger("cassto.stallLimit", 200);

    public static double bestEstimatedMakespan = 0.0;

    private Map<Task, Boolean> urgent;
    private int[] urgentIdx;
    private List<Integer>[] vmNeighbors;

    private int brownianCount = 0;
    private int levyCount = 0;
    private int brownianWins = 0;
    private int levyWins = 0;
    private int brownianNeutral = 0;
    private int levyNeutral = 0;
    private int dedupeCount = 0;
    private long totalLevySteps = 0;

    @Override
    public void run() {
        prepare();

        computeVmNeighbors(m);

        List<Task> baseTopoOrder = decodeOrder(new int[n]);
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
        brownianNeutral = 0;
        levyNeutral = 0;
        dedupeCount = 0;
        totalLevySteps = 0;

        Log.printLine("CASSTO planning: " + n + " tasks, " + m + " VMs, "
                + urgentIdx.length + " urgent tasks, budget " + EVAL_BUDGET + " evals, seed " + SEED);

        // -- 1. Initialization (Algorithm 1: Lines 1-6) --
        Individual[] population = new Individual[POP_SIZE];
        boolean[] isRegenerated = new boolean[POP_SIZE];

        for (int i = 0; i < POP_SIZE; i++) {
            population[i] = randomIndividual();
            evaluate(population[i]);
        }

        Individual best = bestOf(Arrays.asList(population)).cloneIndividual();
        double bestFitness = best.fitness;

        // -- 2. Main search loop (Algorithm 1: Lines 8-34), stops on evaluation budget
        int iter = 0;
        int sinceFresh = 0;
        int maxIter = 10 * EVAL_BUDGET;
        while (budgetLeft() && iter < maxIter) {
            int before = evalCount;
            double pBrownian = P_BROWNIAN_MIN
                    + (P_BROWNIAN_MAX - P_BROWNIAN_MIN) * Math.min(1.0, evalCount / (double) EVAL_BUDGET);

            for (int i = 0; i < POP_SIZE; i++) {
                if (!budgetLeft())
                    break;

                boolean brownian = (rnd.nextDouble() < pBrownian);
                Individual candidate = population[i].cloneIndividual();

                if (brownian) {
                    brownianMove(candidate, m);
                } else {
                    levyMove(candidate, m);
                }

                boolean attempted = candidate.dirty;
                evaluate(candidate);
                boolean wasRegen = isRegenerated[i];
                isRegenerated[i] = false;

                if (attempted && !wasRegen) {
                    if (brownian)
                        brownianCount++;
                    else
                        levyCount++;
                }
                if (candidate.fitness < population[i].fitness - 1e-9) {
                    if (attempted && !wasRegen) {
                        if (brownian)
                            brownianWins++;
                        else
                            levyWins++;
                    }
                    population[i] = candidate;
                } else if (attempted && !wasRegen && Math.abs(candidate.fitness - population[i].fitness) <= 1e-9) {
                    if (brownian)
                        brownianNeutral++;
                    else
                        levyNeutral++;
                }
                if (population[i].fitness < bestFitness) {
                    bestFitness = population[i].fitness;
                    best = population[i].cloneIndividual();
                }
            }

            // Sort population for diversity injection and deduplication
            Integer[] order = sortIndices(population);

            // Deduplicate population
            if (DEDUPE) {
                Set<String> seen = new HashSet<>();
                for (int idx : order) {
                    String sig = signature(population[idx]);
                    if (!seen.add(sig)) {
                        if (!budgetLeft())
                            break;
                        population[idx] = randomIndividual();
                        evaluate(population[idx]);
                        if (population[idx].fitness < bestFitness) {
                            bestFitness = population[idx].fitness;
                            best = population[idx].cloneIndividual();
                        }
                        isRegenerated[idx] = true;
                        dedupeCount++;
                    }
                }
            }

            // Diversity injection: regenerate worst individuals randomly
            int rCount = 2;
            for (int r = 0; r < rCount; r++) {
                if (!budgetLeft())
                    break;
                int worstIdx = order[POP_SIZE - 1 - r];
                if (isRegenerated[worstIdx])
                    continue;
                population[worstIdx] = randomIndividual();
                evaluate(population[worstIdx]);

                if (population[worstIdx].fitness < bestFitness) {
                    bestFitness = population[worstIdx].fitness;
                    best = population[worstIdx].cloneIndividual();
                }
                isRegenerated[worstIdx] = true;
            }

            sinceFresh = (evalCount == before) ? sinceFresh + 1 : 0;
            if (sinceFresh >= STALL_LIMIT) {
                Log.printLine("CASSTO stopped early: " + STALL_LIMIT
                        + " consecutive iterations without fresh evaluations (stagnation).");
                break;
            }

            if (iter % 10 == 0) {
                logProgress("CASSTO", iter, best);
                double avgLevyStep = levyCount > 0 ? (double) totalLevySteps / levyCount : 0.0;
                double bWinRate = brownianCount > 0 ? 100.0 * brownianWins / brownianCount : 0.0;
                double lWinRate = levyCount > 0 ? 100.0 * levyWins / levyCount : 0.0;
                Log.printLine("CASSTO iter " + iter + " | best fit: " + String.format("%.4f", bestFitness)
                        + " | pBrownian: " + String.format("%.2f", pBrownian)
                        + " | Brownian moves: " + brownianCount + " (wins: " + brownianWins + ", neutral: "
                        + brownianNeutral + ", "
                        + String.format("%.1f%%", bWinRate) + ")"
                        + " | Levy moves: " + levyCount + " (wins: " + levyWins + ", neutral: " + levyNeutral + ", "
                        + String.format("%.1f%%", lWinRate) + ", avgStep: " + String.format("%.2f", avgLevyStep) + ")"
                       );
            }
            iter++;
        }

        // -- 3. Apply the best mapping + execution order priority found --
        applySolution(best);
        bestEstimatedMakespan = best.rawMetrics[0];

        double avgLevyStep = levyCount > 0 ? (double) totalLevySteps / levyCount : 0.0;
        double bWinRate = brownianCount > 0 ? 100.0 * brownianWins / brownianCount : 0.0;
        double lWinRate = levyCount > 0 ? 100.0 * levyWins / levyCount : 0.0;
        Log.printLine("  -> Brownian moves (local/nearby VM): " + brownianCount + " (improvements: " + brownianWins
                + ", neutral: " + brownianNeutral
                + ", winRate: " + String.format("%.1f%%", bWinRate) + ")"
                + "\n  -> Levy moves (global/any VM):       " + levyCount + " (improvements: " + levyWins
                + ", neutral: " + levyNeutral
                + ", winRate: " + String.format("%.1f%%", lWinRate) + ", avgStep: " + String.format("%.2f", avgLevyStep)
                + ")"
                + "\n  -> Deduplications: " + dedupeCount);
        logFinal("CASSTO", best, iter);
    }

    private void computeVmNeighbors(int m) {
        vmNeighbors = new List[m];
        for (int v = 0; v < m; v++) {
            vmNeighbors[v] = new ArrayList<>();
            for (int u = 0; u < m; u++) {
                if (u == v)
                    continue;
                if (u != v && dcOfVm(u) == dcOfVm(v)
                        && Math.max(vmMips(u), vmMips(v)) / Math.min(vmMips(u), vmMips(v)) <= 2.0) {
                    vmNeighbors[v].add(u);
                }
            }
            if (vmNeighbors[v].isEmpty()) {
                vmNeighbors[v].add(v);
            }
        }
    }

    private void brownianMove(Individual ind, int vmCount) {
        int moves = Math.max(1, Math.min(5, n / 25)); // small Omega set

        for (int k = 0; k < moves; k++) {
            if (rnd.nextBoolean() && n > 1) {
                // (a) Order move: orderSwapMove(ind)
                orderSwapMove(ind);
            } else {
                // (b) Nearby VM move: same datacenter and same or adjacent speed tier (true
                // refinement)
                int tIdx = rnd.nextInt(n);
                int curVm = ind.vm[tIdx];
                List<Integer> neighbors = vmNeighbors[curVm];
                int newVm = neighbors.get(rnd.nextInt(neighbors.size()));
                if (newVm != curVm) {
                    ind.vm[tIdx] = newVm;
                    ind.dirty = true;
                }
            }
        }
    }

    private void levyMove(Individual ind, int vmCount) {
        double step = LEVY_ALPHA * Math.abs(levySample());
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
                ind.vm[idx] = differentVm(ind.vm[idx]);
            }
            ind.dirty = true;
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
                int subsetSize = Math.min(Math.max(2, moves), vmTasks.size());
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
                ind.vm[idx] = differentVm(ind.vm[idx]);
            }
            ind.dirty = true;
        }
    }



    // Urgency indicator U_i (Paper Eq. 6 & Critical Path Membership)
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

        // 4. Mark tasks as urgent if their path is within 5% of the critical path
        // length
        Map<Task, Boolean> urgencyMap = new HashMap<>();
        double threshold = (1.0 - 0.05) * criticalPathLength;
        for (Task t : order) {
            double totalPath = tl.get(t) + bl.get(t);
            urgencyMap.put(t, totalPath >= threshold);
        }
        return urgencyMap;
    }

    /** Precomputed Mantegna scale parameter sigma for Levy distribution. */
    private static double computeSigma(double lambda) {
        return Math.pow(
                (gamma(1 + lambda) * Math.sin(Math.PI * lambda / 2.0))
                        / (gamma((1 + lambda) / 2.0) * lambda * Math.pow(2.0, (lambda - 1.0) / 2.0)),
                1.0 / lambda);
    }

    /** Mantegna's algorithm for a symmetric Levy-stable step using precomputed sigma. */
    private double levySample() {
        double u = rnd.nextGaussian() * SIGMA_LEVY;
        double v = rnd.nextGaussian();
        return u / Math.pow(Math.abs(v), INV_LEVY_LAMBDA);
    }

    /** Lanczos approximation of the Gamma function. */
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

    private Integer[] sortIndices(Individual[] pop) {
        Integer[] idx = new Integer[pop.length];
        for (int i = 0; i < idx.length; i++) {
            idx[i] = i;
        }
        Arrays.sort(idx, Comparator.comparingDouble(i -> pop[i].fitness));
        return idx;
    }
}