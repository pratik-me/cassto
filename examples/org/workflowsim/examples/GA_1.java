package org.workflowsim.examples;

import java.io.File;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.LinkedList;
import java.util.List;

import org.cloudbus.cloudsim.Cloudlet;
import org.cloudbus.cloudsim.CloudletSchedulerSpaceShared;
import org.cloudbus.cloudsim.DatacenterCharacteristics;
import org.cloudbus.cloudsim.Host;
import org.cloudbus.cloudsim.Log;
import org.cloudbus.cloudsim.Pe;
import org.cloudbus.cloudsim.Storage;
import org.cloudbus.cloudsim.VmAllocationPolicySimple;
import org.cloudbus.cloudsim.VmSchedulerTimeShared;
import org.cloudbus.cloudsim.core.CloudSim;
import org.cloudbus.cloudsim.provisioners.BwProvisionerSimple;
import org.cloudbus.cloudsim.provisioners.PeProvisionerSimple;
import org.cloudbus.cloudsim.provisioners.RamProvisionerSimple;
import org.workflowsim.CondorVM;
import org.workflowsim.WorkflowDatacenter;
import org.workflowsim.WorkflowEngine;
import org.workflowsim.WorkflowPlanner;
import org.workflowsim.utils.ClusteringParameters;
import org.workflowsim.utils.OverheadParameters;
import org.workflowsim.utils.Parameters;
import org.workflowsim.utils.ReplicaCatalog;

/**
 * Runs a workflow using the GA planning algorithm, for direct comparison
 * against CASSTOPlanningAlgorithmExample and HICAPlanningAlgorithmExample
 * (same VM setup, same DAX file, same printing format via ExamplePrinter).
 *
 * IMPORTANT: before this compiles you must have already:
 *   1. Added GAPlanningAlgorithm.java to org.workflowsim.planning
 *   2. Added GA to Parameters.PlanningAlgorithm enum
 *   3. Added a "case GA:" branch in WorkflowPlanner's getPlanningAlgorithm()
 *   4. Added ExamplePrinter.java to org.workflowsim.examples
 */
public class GA_1 {

    public static void main(String[] args) {
        try {
            int vmNum = 20; // keep identical to CASSTO/HICA examples for a fair comparison

            // CHANGE THIS to the same DAX file used in your CASSTO/HICA runs
            String daxPath = "config/dax/Montage_1000.xml";
            File daxFile = new File(daxPath);
            if (!daxFile.exists()) {
                Log.printLine("Warning: replace daxPath with the real path in your environment!");
                return;
            }

            Parameters.SchedulingAlgorithm sch_method = Parameters.SchedulingAlgorithm.STATIC;
            Parameters.PlanningAlgorithm pln_method = Parameters.PlanningAlgorithm.GA;
            ReplicaCatalog.FileSystem file_system = ReplicaCatalog.FileSystem.LOCAL;

            OverheadParameters op = new OverheadParameters(0, null, null, null, null, 0);
            ClusteringParameters.ClusteringMethod method = ClusteringParameters.ClusteringMethod.NONE;
            ClusteringParameters cp = new ClusteringParameters(0, 0, method, null);

            Parameters.init(vmNum, daxPath, null, null, op, cp, sch_method, pln_method, null, 0);
            ReplicaCatalog.init(file_system);

            int numUser = 1;
            Calendar calendar = Calendar.getInstance();
            boolean traceFlag = false;
            CloudSim.init(numUser, calendar, traceFlag);

            WorkflowDatacenter datacenter0 = createDatacenter("Datacenter_0");

            WorkflowPlanner wfPlanner = new WorkflowPlanner("planner_0", 1);
            WorkflowEngine wfEngine = wfPlanner.getWorkflowEngine();

            List<CondorVM> vmList = createVM(wfEngine.getSchedulerId(0), vmNum);
            wfEngine.submitVmList(vmList, 0);
            wfEngine.bindSchedulerDatacenter(datacenter0.getId(), 0);

            CloudSim.startSimulation();
            List<Cloudlet> outputList = wfEngine.getJobsReceivedList();
            CloudSim.stopSimulation();

//            printJobList(outputList);
            printResults(outputList);

        } catch (Exception e) {
            Log.printLine("Error: " + e.getMessage());
            e.printStackTrace();
        }
    }
    
    private static void printResults(List<Cloudlet> list) {
        Log.printLine("========== GENETIC ALGORITHM RESULTS ==========");
        double makespan = 0;
        for (Cloudlet c : list) {
            makespan = Math.max(makespan, c.getFinishTime());
        }
        Log.printLine("Number of jobs: " + list.size());
        Log.printLine("Makespan: " + makespan);
    }

    private static List<CondorVM> createVM(int userId, int vmNum) {
        LinkedList<CondorVM> list = new LinkedList<>();
        long size = 10000;
        int ram = 512;
        long bw = 1000;
        int pesNumber = 1;
        String vmm = "Xen";

        for (int i = 0; i < vmNum; i++) {
            double mips = 1000.0 * (1 + (i % 3)); // same 1000/2000/3000 MIPS pattern as CASSTO/HICA examples
            CondorVM vm = new CondorVM(i, userId, mips, pesNumber, ram, bw, size, vmm,
                    new CloudletSchedulerSpaceShared());
            list.add(vm);
        }
        return list;
    }

    private static WorkflowDatacenter createDatacenter(String name) throws Exception {
        List<Host> hostList = new ArrayList<>();
        List<Pe> peList = new ArrayList<>();

        int mips = 3000;
        int peCount = 8;
        for (int i = 0; i < peCount; i++) {
            peList.add(new Pe(i, new PeProvisionerSimple(mips)));
        }

        int hostId = 0;
        int ram = 16384;
        long storage = 1_000_000;
        long bw = 100_000;

        hostList.add(new Host(
                hostId,
                new RamProvisionerSimple(ram),
                new BwProvisionerSimple(bw),
                storage,
                peList,
                new VmSchedulerTimeShared(peList)));

        String arch = "x86";
        String os = "Linux";
        String vmm = "Xen";
        double timeZone = 10.0;
        double cost = 3.0;
        double costPerMem = 0.05;
        double costPerStorage = 0.001;
        double costPerBw = 0.0;
        LinkedList<Storage> storageList = new LinkedList<>();

        DatacenterCharacteristics characteristics = new DatacenterCharacteristics(
                arch, os, vmm, hostList, timeZone, cost, costPerMem, costPerStorage, costPerBw);

        return new WorkflowDatacenter(
                name, characteristics, new VmAllocationPolicySimple(hostList), storageList, 0);
    }
}