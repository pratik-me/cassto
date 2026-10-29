package org.workflowsim.planning;

import java.util.*;

import org.cloudbus.cloudsim.Log;
import org.workflowsim.Task;

public class GAPlanningAlgorithm extends OrderAwarePlanningBase {

    private static final int POP_SIZE = Integer.getInteger("ga.pop", 30);
    private static final int TOURNAMENT_SIZE = Integer.getInteger("ga.tournament", 3);
    private static final double CROSSOVER_PROB = dbl("ga.pc", 0.8);
    private static final double MUTATION_PROB = dbl("ga.pm", 0.05);
    private static final boolean MUT_PER_GENE = Boolean.parseBoolean(System.getProperty("ga.mutPerGene", "false"));
    private static final boolean DEDUPE = Boolean.parseBoolean(System.getProperty("ga.dedupe", "true"));

    @Override
    public void run() {
        prepare();
        Log.printLine("GA planning: " + n + " tasks, " + m + " VMs, pop " + POP_SIZE
                + ", pc " + CROSSOVER_PROB + ", pm " + MUTATION_PROB
                + (MUT_PER_GENE ? " (per gene)" : " (per offspring)")
                + ", budget " + EVAL_BUDGET + " evals, seed " + SEED);

        // -- Initialization (Algorithm 2 L1-7) --
        List<Individual> pop = new ArrayList<>(POP_SIZE);
        for (int i = 0; i < POP_SIZE; i++) {
            Individual ind = randomIndividual();
            evaluate(ind);
            pop.add(ind);
        }
        Individual best = bestOf(pop).cloneIndividual();

        // -- Generation loop (L8-37), stops on evaluation budget --
        int gen = 0;
        int genCap = 10 * EVAL_BUDGET;     // safety net if pc and pm are ~0 and no evaluations happen
        while (budgetLeft() && gen < genCap) {
            List<Individual> kids = breed(pop);

            // L26-30: repair + evaluate.
            List<Individual> evaluated = new ArrayList<>(kids.size());
            for (Individual k : kids) {
                if (k.dirty && !budgetLeft()) continue;   // budget exhausted: discard unevaluated offspring
                evaluate(k);
                evaluated.add(k);
            }

            pop = selectSurvivors(pop, evaluated);       // L31-33

            Individual top = pop.get(0);                 // L34-36
            if (top.fitness < best.fitness) best = top.cloneIndividual();

            if (gen % 10 == 0) logProgress("GA", gen, best);
            gen++;
        }

        applySolution(best);
        logFinal("GA", best, gen);
    }

    // =====================================================================
    // Variation
    // =====================================================================

    private List<Individual> breed(List<Individual> pop) {
        List<Individual> kids = new ArrayList<>(POP_SIZE);
        while (kids.size() < POP_SIZE) {
            Individual a = tournament(pop);
            Individual b = tournament(pop);

            Individual c1, c2;
            if (rnd.nextDouble() < CROSSOVER_PROB) {
                Individual[] c = crossover(a, b);
                c1 = c[0];
                c2 = c[1];
            } else {
                c1 = a.cloneIndividual();         // L17
                c2 = b.cloneIndividual();
            }
            mutate(c1);
            mutate(c2);

            kids.add(c1);
            if (kids.size() < POP_SIZE) kids.add(c2);
        }
        return kids;
    }

    private Individual tournament(List<Individual> pop) {
        Individual best = null;
        for (int k = 0; k < TOURNAMENT_SIZE; k++) {
            Individual c = pop.get(rnd.nextInt(pop.size()));
            if (best == null || c.fitness < best.fitness) best = c;
        }
        return best;
    }

    /** Two-point crossover on VM genes + order crossover (OX) on the execution order. */
    private Individual[] crossover(Individual a, Individual b) {
        Individual c1 = a.cloneIndividual();
        Individual c2 = b.cloneIndividual();

        // VM genes (indexed by task): swap a segment
        int p1 = rnd.nextInt(n), p2 = rnd.nextInt(n);
        int lo = Math.min(p1, p2), hi = Math.max(p1, p2);
        for (int t = lo; t <= hi; t++) {
            int tmp = c1.vm[t];
            c1.vm[t] = c2.vm[t];
            c2.vm[t] = tmp;
        }

        // Execution order (indexed by position): OX with its own cut points
        int q1 = rnd.nextInt(n), q2 = rnd.nextInt(n);
        int qlo = Math.min(q1, q2), qhi = Math.max(q1, q2);
        c1.priority = oxPriority(a, b, qlo, qhi);
        c2.priority = oxPriority(b, a, qlo, qhi);

        // OX can violate precedence; Kahn repairs it. Re-decode now so order is consistent for mutation.
        redecode(c1);
        redecode(c2);
        c1.dirty = true;
        c2.dirty = true;
        return new Individual[] { c1, c2 };
    }

    /** OX: keep first parent's segment [lo..hi] by position, fill the rest in the second parent's order. */
    private int[] oxPriority(Individual first, Individual second, int lo, int hi) {
        int[] child = new int[n];
        boolean[] used = new boolean[n];
        for (int pos = lo; pos <= hi; pos++) {
            int ti = taskIndexMap.get(first.order.get(pos));
            child[pos] = ti;
            used[ti] = true;
        }
        int fill = 0;
        for (Task t : second.order) {
            int ti = taskIndexMap.get(t);
            if (used[ti]) continue;
            while (fill >= lo && fill <= hi) fill++;
            child[fill++] = ti;
        }
        int[] prio = new int[n];
        for (int pos = 0; pos < n; pos++) prio[child[pos]] = pos;
        return prio;
    }

    /** Algorithm 2 L21-23: (a) swap in execution order, or (b) reassign a task to a DIFFERENT VM. */
    private void mutate(Individual ind) {
        int trials = MUT_PER_GENE ? n : 1;
        for (int g = 0; g < trials; g++) {
            if (rnd.nextDouble() >= MUTATION_PROB) continue;
            if (rnd.nextBoolean() && n > 1) {
                orderSwapMove(ind);              // shared with CASSTO; -DsameVmSwap
            } else {
                reassignToDifferentVm(ind, rnd.nextInt(n));
            }
        }
    }

    private List<Individual> selectSurvivors(List<Individual> parents, List<Individual> kids) {
        List<Individual> all = new ArrayList<>(parents.size() + kids.size());
        all.addAll(parents);
        all.addAll(kids);
        all.sort(Comparator.comparingDouble((Individual x) -> x.fitness));   // List.sort is stable

        if (!DEDUPE) {
            return new ArrayList<>(all.subList(0, Math.min(POP_SIZE, all.size())));
        }

        List<Individual> out = new ArrayList<>(POP_SIZE);
        List<Individual> dupes = new ArrayList<>();
        Set<String> seen = new HashSet<>();
        for (Individual x : all) {
            if (out.size() == POP_SIZE) break;
            if (seen.add(signature(x))) out.add(x);
            else dupes.add(x);
        }
        for (Individual x : dupes) {   // only if there were too few distinct schedules
            if (out.size() == POP_SIZE) break;
            out.add(x);
        }
        out.sort(Comparator.comparingDouble((Individual x) -> x.fitness));
        return out;
    }
}