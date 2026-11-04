package org.workflowsim.planning;

import java.util.*;

import org.cloudbus.cloudsim.Log;
import org.workflowsim.Task;

public class CASSTOPlanningAlgorithm extends OrderAwarePlanningBase {
    private static final int POP_SIZE = 30;
    private static final double DIVERSITY_THRESHOLD = Double.parseDouble(System.getProperty("cassto.delta", "0.25"));
    private static final double LEVY_LAMBDA = Math.min(1.9, Math.max(0.3,
        Double.parseDouble(System.getProperty("cassto.lambda", "1.5"))));
    private static final double SIGMA_LEVY = computeSigma(LEVY_LAMBDA);
    private static final double INV_LEVY_LAMBDA = 1.0 / LEVY_LAMBDA;
    private static final boolean USE_SCHEDULE = Boolean.parseBoolean(System.getProperty("cassto.useSchedule", "false"));
    private static final boolean INVERT_DIVERSITY = Boolean.parseBoolean(System.getProperty("cassto.invertDiversity", "false"));
    // Movement Mode: "urgency", "schedule", "diversity"
    private static final String PHASE_MODE = System.getProperty("cassto.mode",
            Boolean.getBoolean("cassto.useSchedule") ? "schedule" : "urgency");
    // Task Pool: "none", "urgent" (partition urgent for Levy, normal for Brownian)
    private static final String TASK_POOL = System.getProperty("cassto.pool",
            "urgency".equalsIgnoreCase(PHASE_MODE) ? "urgent" : "none");

    private static final double P_BROWNIAN_MIN = 0.2;
    private static final double P_BROWNIAN_MAX = 0.8;

    private static final double LEVY_ALPHA = Double.parseDouble(System.getProperty("cassto.alpha", "2.0"));

    // Adaptive Parameter Tuning via Learning Models (Paper Section 7)
    private static final boolean ADAPTIVE_TUNING = Boolean.parseBoolean(System.getProperty("cassto.adaptive", "false"));
    private static final boolean ADAPTIVE_INIT = Boolean.parseBoolean(System.getProperty("cassto.adaptiveInit", System.getProperty("cassto.adaptive", "false")));
    private static final boolean BROWNIAN_DECAY = Boolean.parseBoolean(System.getProperty("cassto.brownianDecay", "false"));
    private static final double BROWNIAN_DECAY_FROM = Double.parseDouble(System.getProperty("cassto.brownianDecayFrom", "8.0"));
    private static final double BROWNIAN_DECAY_TO = Double.parseDouble(System.getProperty("cassto.brownianDecayTo", "1.0"));
    private static final double ALPHA_MIN = 0.5, ALPHA_MAX = 6.0;
    private static final double BROWNIAN_MIN = 1.0;
    private static final int WINDOW = 10, MIN_ATTEMPTS = 20;
    private static final double TARGET_RATE = 0.10, ETA = 1.0;
    private static final double BROWNIAN_TARGET_RATE = Double.parseDouble(System.getProperty("cassto.brownianTarget", "0.15"));
    private static final int BURST_AFTER = 8;
    private boolean alphaBurstDone = false;
    private boolean brownianBurstDone = false;
    private double currentLevyAlpha = LEVY_ALPHA;
    private double currentBrownianStep = 1.0;
    private double brownianMax = 4.0;
    private double currentDiversityThreshold = DIVERSITY_THRESHOLD;
    private double initialDiversity = 0.95;
    private double currentUrgencyThreshold = 1.0;
    private int currentBrownianMoves = 1;
    private double vmSpeedRatioThreshold = 2.0;

    // Reinforcement learning feedback tracking
    private int windowBrownianWins = 0;
    private int windowLevyWins = 0;
    private int windowBrownianAttempts = 0;
    private int windowLevyAttempts = 0;
    private int stagnationCount = 0;
    private double prevBestRaw = Double.POSITIVE_INFINITY;
    private List<Task> baseTopoOrder;

    private static final boolean DEDUPE = Boolean.parseBoolean(System.getProperty("cassto.dedupe", "true"));
    private static final int STALL_LIMIT = Integer.getInteger("cassto.stallLimit", 200);
    private static final int REGEN_COUNT = Integer.getInteger("cassto.regenCount", 0);

    public static double bestEstimatedMakespan = 0.0;

    private Map<Task, Boolean> urgent;
    private int[] urgentIdx;
    private int[] normalIdx;
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
        bestEstimatedMakespan = 0.0;

        computeVmNeighbors(m);

        baseTopoOrder = decodeOrder(new int[n]);
        
        // Identify initial critical path tasks (Paper Eq. 6)
        urgent = computeUrgency(baseTopoOrder, 1.0);
        refreshUrgentIndices();

        brownianMax = Math.max(4.0, (double) n / 10.0);
        int customBMoves = Integer.getInteger("cassto.brownianMoves", -1);
        if (customBMoves > 0) {
            currentBrownianStep = customBMoves;
        } else {
            currentBrownianStep = Math.max(1.0, Math.min(4.0, (double) n / 25.0));
        }
        currentBrownianMoves = (int) Math.round(currentBrownianStep);
        alphaBurstDone = false;
        brownianBurstDone = false;
        if (ADAPTIVE_INIT) {
            // Adaptive initial parameters based on DAG properties
            boolean isParallel = (urgentIdx.length < n / 3);
            currentDiversityThreshold = isParallel ? Math.min(0.35, DIVERSITY_THRESHOLD * 1.2) : DIVERSITY_THRESHOLD;
            currentUrgencyThreshold = isParallel ? 0.999 : 1.0;
            currentLevyAlpha = isParallel ? 2.5 : 1.8;
            urgent = computeUrgency(baseTopoOrder, currentUrgencyThreshold);
            refreshUrgentIndices();
            if (urgentIdx.length == n || urgentIdx.length == 0) {
                currentUrgencyThreshold = 1.0;
                urgent = computeUrgency(baseTopoOrder, 1.0);
                refreshUrgentIndices();
            }
        } else {
            currentDiversityThreshold = DIVERSITY_THRESHOLD;
            currentUrgencyThreshold = 1.0;
            currentLevyAlpha = LEVY_ALPHA;
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
                + urgentIdx.length + " urgent tasks, " + normalIdx.length + " normal tasks, mode " + PHASE_MODE + ", pool " + TASK_POOL + ", budget " + EVAL_BUDGET + " evals, delta " + DIVERSITY_THRESHOLD + ", lambda " + LEVY_LAMBDA + ", seed " + SEED);

        // -- 1. Initialization (Algorithm 1: Lines 1-6) --
        Individual[] population = new Individual[POP_SIZE];
        boolean[] isRegenerated = new boolean[POP_SIZE];

        for (int i = 0; i < POP_SIZE; i++) {
            population[i] = randomIndividual();
            repair(population[i]);
            population[i].rawMetrics = computeRawMetrics(population[i]);
            evalCount++;
            population[i].dirty = false;
        }
        refreshBounds(population, null);
        initialDiversity = populationDiversity(population);

        Individual best = bestOf(Arrays.asList(population)).cloneIndividual();
        double bestFitness = best.fitness;
        prevBestRaw = best.rawMetrics[0];
        stagnationCount = 0;
        alphaBurstDone = false;

        // -- 2. Main search loop (Algorithm 1)
        int iter = 0;
        int sinceFresh = 0;
        int maxIter = 10 * EVAL_BUDGET;
        while (budgetLeft() && iter < maxIter) {
            refreshBounds(population, best);
            if (BROWNIAN_DECAY) {
                double progress = Math.min(1.0, (double) evalCount / (double) EVAL_BUDGET);
                currentBrownianStep = BROWNIAN_DECAY_FROM - (BROWNIAN_DECAY_FROM - BROWNIAN_DECAY_TO) * progress;
                currentBrownianMoves = Math.max((int) Math.round(BROWNIAN_DECAY_TO), (int) Math.round(currentBrownianStep));
            }
            bestFitness = best.fitness;

            int before = evalCount;
            double d = populationDiversity(population);
            double relD = d / Math.max(1e-9, initialDiversity);
            

            // Track stagnation across iterations using scale-invariant raw makespan
            double bestRaw = best.rawMetrics[0];
            if (bestRaw < prevBestRaw - 1e-6) {
                stagnationCount = 0;
                alphaBurstDone = false;
                brownianBurstDone = false;
                prevBestRaw = bestRaw;
            } else {
                stagnationCount++;
            }


            boolean isUrgencyMode = "urgency".equalsIgnoreCase(PHASE_MODE);
            boolean isScheduleMode = "schedule".equalsIgnoreCase(PHASE_MODE) || USE_SCHEDULE;
            boolean brownianPhase = false;
            if (isScheduleMode) {
                double pBrownian = P_BROWNIAN_MIN + (P_BROWNIAN_MAX - P_BROWNIAN_MIN) * Math.min(1.0, evalCount / (double) EVAL_BUDGET);
                brownianPhase = (rnd.nextDouble() < pBrownian);
            } else if (!isUrgencyMode) {
                // Diversity mode
                if (DIVERSITY_THRESHOLD <= 0.5) {
                    
                    brownianPhase = (relD >= currentDiversityThreshold) && !(stagnationCount > 10 && relD < 0.99);
                } else {
                    brownianPhase = INVERT_DIVERSITY ? (d < currentDiversityThreshold) : (d >= currentDiversityThreshold);
                }
            }

            for (int i = 0; i < POP_SIZE; i++) {
                if (!budgetLeft())
                    break;
                Individual candidate = population[i].cloneIndividual();

                boolean isLevyMove = false;
                if (isUrgencyMode) {
                    int t = (urgentIdx.length > 0 && normalIdx.length > 0)
                            ? (rnd.nextBoolean() ? urgentIdx[rnd.nextInt(urgentIdx.length)] : normalIdx[rnd.nextInt(normalIdx.length)])
                            : rnd.nextInt(n);
                    boolean isUrgent = (urgent != null && urgent.containsKey(taskList.get(t)))
                            ? urgent.get(taskList.get(t))
                            : false;
                    if (isUrgent) {
                        isLevyMove = true;
                        levyTaskMove(candidate, t, m);
                    } else {
                        isLevyMove = false;
                        brownianTaskMove(candidate, t, m);
                    }
                } else {
                    if (brownianPhase) {
                        isLevyMove = false;
                        brownianMove(candidate, m);
                    } else {
                        isLevyMove = true;
                        levyMove(candidate, m);
                    }
                }

                boolean attempted = candidate.dirty;
                evaluate(candidate);
                boolean wasRegen = isRegenerated[i];
                isRegenerated[i] = false;

                if (attempted && !wasRegen) {
                    if (!isLevyMove) {
                        brownianCount++;
                        windowBrownianAttempts++;
                    } else {
                        levyCount++;
                        windowLevyAttempts++;
                    }
                }
                if (candidate.fitness < population[i].fitness - 1e-9) {
                    if (attempted && !wasRegen) {
                        if (!isLevyMove) {
                            brownianWins++;
                            windowBrownianWins++;
                        } else {
                            levyWins++;
                            windowLevyWins++;
                        }
                    }
                    updateMetricBounds(candidate.rawMetrics[0], candidate.rawMetrics[1], candidate.rawMetrics[2]);
                    population[i] = candidate;
                } else if (attempted && !wasRegen && Math.abs(candidate.fitness - population[i].fitness) <= 1e-9) {
                    if (!isLevyMove)
                        brownianNeutral++;
                    else
                        levyNeutral++;
                }
                if (population[i].fitness < bestFitness) {
                    bestFitness = population[i].fitness;
                    best = population[i].cloneIndividual();
                }
            }

            // Sort population for deduplication (best to worst to preserve superior copies)
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

            // Diversity injection
            if (REGEN_COUNT > 0) {
                Integer[] freshOrder = sortIndices(population);
                for (int r = 0; r < REGEN_COUNT; r++) {
                    if (!budgetLeft())
                        break;
                    int worstIdx = freshOrder[POP_SIZE - 1 - r];
                    population[worstIdx] = randomIndividual();
                    evaluate(population[worstIdx]);

                    if (population[worstIdx].fitness < bestFitness) {
                        bestFitness = population[worstIdx].fitness;
                        best = population[worstIdx].cloneIndividual();
                    }
                    isRegenerated[worstIdx] = true;
                }
            }

            sinceFresh = (evalCount == before) ? sinceFresh + 1 : 0;
            if (sinceFresh >= STALL_LIMIT) {
                Log.printLine("CASSTO stopped early: " + STALL_LIMIT
                        + " consecutive iterations without fresh evaluations (stagnation).");
                break;
            }
            // Adaptive Parameter Tuning (Paper Section 7)
            if (ADAPTIVE_TUNING && (iter + 1) % WINDOW == 0) {
                double progress = Math.min(1.0, (double) evalCount / EVAL_BUDGET);

                // 1. Dynamic step size adaptation (Levy alpha)
                if (windowLevyAttempts >= MIN_ATTEMPTS) {
                    double rate = (double) windowLevyWins / windowLevyAttempts;
                    currentLevyAlpha *= Math.exp(ETA * (rate - TARGET_RATE));
                } // if no data, leave alpha alone

                if (stagnationCount >= BURST_AFTER && !alphaBurstDone) { // one-shot burst, not every iteration
                    currentLevyAlpha *= 1.5;
                    alphaBurstDone = true;
                }
                currentLevyAlpha = Math.max(ALPHA_MIN, Math.min(ALPHA_MAX, currentLevyAlpha));

                // 2. Dynamic Brownian step count adaptation (symmetric to Levy alpha)
                double bRate = windowBrownianAttempts > 0 ? (double) windowBrownianWins / windowBrownianAttempts : 0.0;
                if (windowBrownianAttempts >= MIN_ATTEMPTS) {
                    currentBrownianStep *= Math.exp(ETA * (bRate - BROWNIAN_TARGET_RATE));
                } // if no data, leave brownian step alone

                if (stagnationCount >= BURST_AFTER && !brownianBurstDone) {
                    currentBrownianStep *= 1.5;
                    brownianBurstDone = true;
                }
                currentBrownianStep = Math.max(BROWNIAN_MIN, Math.min(brownianMax, currentBrownianStep));
                currentBrownianMoves = (int) Math.round(currentBrownianStep);

                if (stagnationCount > 15 && vmSpeedRatioThreshold < 4.0) {
                    vmSpeedRatioThreshold += 0.5;
                    computeVmNeighbors(m, vmSpeedRatioThreshold);
                }

                // 3. Dynamic urgency threshold adaptation
                double targetUrgency = Math.max(0.85, 1.0 - 0.15 * progress);
                if (Math.abs(targetUrgency - currentUrgencyThreshold) >= 0.05) {
                    currentUrgencyThreshold = targetUrgency;
                    urgent = computeUrgency(baseTopoOrder, currentUrgencyThreshold);
                    refreshUrgentIndices();
                }

                // 4. Dynamic diversity threshold adaptation
                double baseDelta = DIVERSITY_THRESHOLD;
                currentDiversityThreshold = Math.max(0.12, baseDelta * (1.0 - 0.45 * progress));
                double divVal = (DIVERSITY_THRESHOLD <= 0.5) ? relD : d;
                if (stagnationCount > 12 && divVal < currentDiversityThreshold) {
                    currentDiversityThreshold = Math.min(0.40, currentDiversityThreshold * 1.25);
                }

                // Window summary logging
                double lRate = windowLevyAttempts > 0 ? (double) windowLevyWins / windowLevyAttempts : 0.0;
                Log.printLine("CASSTO window " + ((iter + 1) / WINDOW)
                        + " (iter " + (iter - WINDOW + 1) + ".." + iter + ")"
                        + " | raw bStep: " + String.format("%.3f", currentBrownianStep)
                        + " (moves: " + currentBrownianMoves + ", cap: " + String.format("%.1f", brownianMax) + ")"
                        + " | bWinRate: " + (windowBrownianAttempts > 0 ? String.format("%.1f%% (%d/%d)", bRate * 100.0, windowBrownianWins, windowBrownianAttempts) : "N/A (0)")
                        + " | alpha: " + String.format("%.3f", currentLevyAlpha)
                        + " | lWinRate: " + (windowLevyAttempts > 0 ? String.format("%.1f%% (%d/%d)", lRate * 100.0, windowLevyWins, windowLevyAttempts) : "N/A (0)")
                        + " | stag: " + stagnationCount);

                windowLevyWins = 0;
                windowLevyAttempts = 0;
                windowBrownianWins = 0;
                windowBrownianAttempts = 0;
            }


            if (iter % 10 == 0) {
                logProgress("CASSTO", iter, best);
                double avgLevyStep = levyCount > 0 ? (double) totalLevySteps / levyCount : 0.0;
                double bWinRate = brownianCount > 0 ? 100.0 * brownianWins / brownianCount : 0.0;
                double lWinRate = levyCount > 0 ? 100.0 * levyWins / levyCount : 0.0;
                Log.printLine("CASSTO iter " + iter + " | best fit: " + String.format("%.4f", bestFitness)
                        + " | mode: " + PHASE_MODE
                        + " | D: " + String.format("%.4f", d)
                        + (ADAPTIVE_TUNING ? (" | alpha: " + String.format("%.2f", currentLevyAlpha) + " | bMoves: " + currentBrownianMoves + " | urg: " + String.format("%.2f", currentUrgencyThreshold)) : "")
                        + " | Brownian: " + brownianCount + " (" + String.format("%.1f%%", bWinRate) + ")"
                        + " | Levy: " + levyCount + " (" + String.format("%.1f%%", lWinRate) + ", avgStep: " + String.format("%.2f", avgLevyStep) + ")"
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
        double finalD = populationDiversity(population);
        Log.printLine("  -> Final diversity D: " + String.format("%.4f", finalD) + " (mode: " + PHASE_MODE + ", delta: " + DIVERSITY_THRESHOLD + ")\n"
                + "  -> Brownian moves (local/nearby VM): " + brownianCount + " (improvements: " + brownianWins
                + ", neutral: " + brownianNeutral
                + ", winRate: " + String.format("%.1f%%", bWinRate) + ")"
                + "\n  -> Levy moves (global/any VM):       " + levyCount + " (improvements: " + levyWins
                + ", neutral: " + levyNeutral
                + ", winRate: " + String.format("%.1f%%", lWinRate) + ", avgStep: " + String.format("%.2f", avgLevyStep)
                + ")"
                + "\n  -> Deduplications: " + dedupeCount);
        logFinal("CASSTO", best, iter);
    }

    private void computeVmNeighbors(int m, double ratioThreshold) {
        vmNeighbors = new List[m];
        for (int v = 0; v < m; v++) {
            vmNeighbors[v] = new ArrayList<>();
            for (int u = 0; u < m; u++) {
                if (u == v)
                    continue;
                if (dcOfVm(u) == dcOfVm(v)
                        && Math.max(vmMips(u), vmMips(v)) / Math.min(vmMips(u), vmMips(v)) <= ratioThreshold) {
                    vmNeighbors[v].add(u);
                }
            }
            if (vmNeighbors[v].isEmpty()) {
                vmNeighbors[v].add(v);
            }
        }
    }

    private void computeVmNeighbors(int m) {
        computeVmNeighbors(m, vmSpeedRatioThreshold);
    }

    private int pickBrownianTask() {
        if ("urgent".equalsIgnoreCase(TASK_POOL) && normalIdx != null && normalIdx.length > 0) {
            return normalIdx[rnd.nextInt(normalIdx.length)];
        }
        return rnd.nextInt(n);
    }

    private int pickLevyTask() {
        if ("urgent".equalsIgnoreCase(TASK_POOL) && urgentIdx != null && urgentIdx.length > 0) {
            return urgentIdx[rnd.nextInt(urgentIdx.length)];
        }
        return rnd.nextInt(n);
    }

    private void brownianMove(Individual ind, int vmCount) {
        int moves = currentBrownianMoves;

        for (int k = 0; k < moves; k++) {
            if (rnd.nextBoolean() && n > 1) {
                // Order move
                orderSwapMove(ind);
            } else {
                // Nearby VM move on normal tasks
                int tIdx = pickBrownianTask();
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

    private void levyOrderMove(Individual ind, int vmCount, int moves) {
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
            int idx = pickLevyTask();
            ind.vm[idx] = differentVm(ind.vm[idx]);
        }
        ind.dirty = true;
    }

    private void levyMove(Individual ind, int vmCount) {
        double alpha = ADAPTIVE_TUNING ? currentLevyAlpha : LEVY_ALPHA;
        double step = alpha * Math.abs(levySample());
        int moves = Math.min(n, Math.max(1, (int) Math.ceil(step)));
        totalLevySteps += moves;

        if (rnd.nextBoolean()) {
            // Global VM jump on urgent tasks
            for (int k = 0; k < moves; k++) {
                int idx = pickLevyTask();
                ind.vm[idx] = differentVm(ind.vm[idx]);
            }
            ind.dirty = true;
        } else {
            levyOrderMove(ind, vmCount, moves);
        }
    }

    // urgent task -> Levy global move
    private void levyTaskMove(Individual ind, int t, int vmCount) {
        double alpha = ADAPTIVE_TUNING ? currentLevyAlpha : LEVY_ALPHA;
        double step = alpha * Math.abs(levySample());
        int moves = Math.min(n, Math.max(1, (int) Math.ceil(step)));
        totalLevySteps += moves;

        if (rnd.nextBoolean()) {
            // Global VM jump on urgent task t (and extra urgent tasks if step > 1)
            ind.vm[t] = differentVm(ind.vm[t]);
            for (int k = 1; k < moves; k++) {
                int idx = pickLevyTask();
                ind.vm[idx] = differentVm(ind.vm[idx]);
            }
            ind.dirty = true;
        } else {
            // (b) Order move among tasks sharing a VM
            levyOrderMove(ind, vmCount, moves);
        }
    }

    private void applySingleBrownianTaskMove(Individual ind, int taskIdx) {
        if (rnd.nextBoolean() && n > 1) {
            // Same-VM order swap
            List<Integer> sameVmTasks = new ArrayList<>();
            for (int i = 0; i < n; i++) {
                if (i != taskIdx && ind.vm[i] == ind.vm[taskIdx]) {
                    sameVmTasks.add(i);
                }
            }
            if (!sameVmTasks.isEmpty()) {
                int other = sameVmTasks.get(rnd.nextInt(sameVmTasks.size()));
                int tmp = ind.priority[taskIdx];
                ind.priority[taskIdx] = ind.priority[other];
                ind.priority[other] = tmp;
                ind.dirty = true;
            } else {
                orderSwapMove(ind);
            }
        } else {
            // Nearby VM move on normal task
            int curVm = ind.vm[taskIdx];
            List<Integer> neighbors = vmNeighbors[curVm];
            int newVm = neighbors.get(rnd.nextInt(neighbors.size()));
            if (newVm != curVm) {
                ind.vm[taskIdx] = newVm;
                ind.dirty = true;
            }
        }
    }

    // Urgency-driven task dispatch: normal task
    private void brownianTaskMove(Individual ind, int t, int vmCount) {
        int moves = currentBrownianMoves;
        applySingleBrownianTaskMove(ind, t);
        for (int k = 1; k < moves; k++) {
            int extra = pickBrownianTask();
            applySingleBrownianTaskMove(ind, extra);
        }
    }



    private void refreshUrgentIndices() {
        List<Integer> u = new ArrayList<>();
        List<Integer> norm = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            if (urgent.get(taskList.get(i))) {
                u.add(i);
            } else {
                norm.add(i);
            }
        }
        urgentIdx = new int[u.size()];
        for (int i = 0; i < urgentIdx.length; i++) {
            urgentIdx[i] = u.get(i);
        }
        normalIdx = new int[norm.size()];
        for (int i = 0; i < normalIdx.length; i++) {
            normalIdx[i] = norm.get(i);
        }
    }

    // Urgency indicator U_i (Paper Eq. 6 & Critical Path Membership with adaptive threshold)
    private Map<Task, Boolean> computeUrgency(List<Task> order, double urgencyThreshold) {
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

        // 4. Paper Eq. 6
        double deadlineFactor = Double.parseDouble(System.getProperty("cassto.deadlineFactor", "0"));
        double eps = 1e-4;
        Map<Task, Boolean> urgencyMap = new HashMap<>();
        for (Task t : order) {
            double totalPath = tl.get(t) + bl.get(t);
            boolean onCriticalPath = (totalPath >= criticalPathLength * urgencyThreshold - eps);
            boolean nearDeadline = (deadlineFactor > 0) && (totalPath >= deadlineFactor * criticalPathLength * 0.95);
            urgencyMap.put(t, onCriticalPath || nearDeadline);
        }
        return urgencyMap;
    }

    // Precomputed Mantegna scale parameter sigma for Levy distribution.
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

    // Population diversity (Paper Eq. 5 / Sec 4.3)
    private double populationDiversity(Individual[] pop) {
        int popSize = pop.length;
        double totalDist = 0.0;
        int pairs = 0;

        for (int i = 0; i < popSize; i++) {
            for (int j = i + 1; j < popSize; j++) {
                int diff = 0;
                for (int t = 0; t < n; t++) {
                    if (pop[i].vm[t] != pop[j].vm[t]) {
                        diff++;
                    }
                }
                totalDist += (double) diff / n;
                pairs++;
            }
        }
        return pairs > 0 ? totalDist / pairs : 0.0;
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