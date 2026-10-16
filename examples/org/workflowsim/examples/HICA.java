package org.workflowsim.examples;

import java.io.File;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

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
import org.workflowsim.ClusterStorage;
import org.workflowsim.CondorVM;
import org.workflowsim.WorkflowDatacenter;
import org.workflowsim.WorkflowEngine;
import org.workflowsim.WorkflowPlanner;
import org.workflowsim.utils.ClusteringParameters;
import org.workflowsim.utils.OverheadParameters;
import org.workflowsim.utils.Parameters;
import org.workflowsim.utils.ReplicaCatalog;

public class HICA {

    public static void main(String[] args) {
        try {
            int vmNum = 20;

            String daxPath = "config/dax/Montage_50.xml";
            File daxFile = new File(daxPath);
            if (!daxFile.exists()) {
                Log.printLine("Warning: replace daxPath with the real path in your environment!");
                return;
            }

            Parameters.SchedulingAlgorithm sch_method = Parameters.SchedulingAlgorithm.STATIC;
            Parameters.PlanningAlgorithm pln_method = Parameters.PlanningAlgorithm.HICA;
            ReplicaCatalog.FileSystem file_system = ReplicaCatalog.FileSystem.SHARED;

            OverheadParameters op = new OverheadParameters(0, null, null, null, null, 0);
            ClusteringParameters.ClusteringMethod method = ClusteringParameters.ClusteringMethod.NONE;
            ClusteringParameters cp = new ClusteringParameters(0, 0, method, null);

            Parameters.init(vmNum, daxPath, null, null, op, cp, sch_method, pln_method, null, 0);
            ReplicaCatalog.init(file_system);

            int numUser = 1;
            Calendar calendar = Calendar.getInstance();
            boolean traceFlag = false;
            CloudSim.init(numUser, calendar, traceFlag);

            WorkflowDatacenter datacenter0 = createDatacenter("Datacenter_0", (vmNum + 1) / 2);
            WorkflowDatacenter datacenter1 = createDatacenter("Datacenter_1", vmNum / 2);

            WorkflowPlanner wfPlanner = new WorkflowPlanner("planner_0", 1);
            WorkflowEngine wfEngine = wfPlanner.getWorkflowEngine();

            List<CondorVM> vmList = createVM(wfEngine.getSchedulerId(0), vmNum);
            wfEngine.submitVmList(vmList, 0);
            wfEngine.bindSchedulerDatacenter(datacenter0.getId(), 0);
            wfEngine.bindSchedulerDatacenter(datacenter1.getId(), 0);

            Map<Integer, String> dcNames = new HashMap<>();
            dcNames.put(datacenter0.getId(), datacenter0.getName());
            dcNames.put(datacenter1.getId(), datacenter1.getName());

            CloudSim.startSimulation();
            List<Cloudlet> outputList = wfEngine.getJobsReceivedList();
            CloudSim.stopSimulation();

            printResults(outputList, vmList, dcNames);

        } catch (Exception e) {
            Log.printLine("Error: " + e.getMessage());
            e.printStackTrace();
        }
    }

    private static List<CondorVM> createVM(int userId, int vmNum) {
        LinkedList<CondorVM> list = new LinkedList<>();
        long size = 10000;
        int ram = 512;
        long bw = 1000;
        int pesNumber = 1;
        String vmm = "Xen";

        for (int i = 0; i < vmNum; i++) {
            double mips = 1000.0 * (1 + (i % 3)); // As per the paper
            CondorVM vm = new CondorVM(i, userId, mips, pesNumber, ram, bw, size, vmm,
                    new CloudletSchedulerSpaceShared());
            list.add(vm);
        }
        return list;
    }
    
    private static void printResults(List<Cloudlet> list, List<CondorVM> vmList, Map<Integer, String> dcNames) {
        Log.printLine("========== HICA RESULTS ==========");
        double makespan = 0;
        Map<Integer, Integer> dcJobCount = new HashMap<>();
        Map<Integer, Integer> vmActualDcId = new HashMap<>();
        for (Cloudlet c : list) {
            makespan = Math.max(makespan, c.getFinishTime());
            dcJobCount.put(c.getResourceId(), dcJobCount.getOrDefault(c.getResourceId(), 0) + 1);
            vmActualDcId.put(c.getVmId(), c.getResourceId());
        }
        Log.printLine("Number of jobs: " + list.size());
        Log.printLine("Makespan: " + makespan);

        Log.printLine("\n========== VM ACTUAL DATACENTER ASSIGNMENTS ==========");
        if (vmList != null) {
            for (CondorVM vm : vmList) {
                int dcId = vmActualDcId.containsKey(vm.getId()) ? vmActualDcId.get(vm.getId()) : (vm.getId() < (vmList.size() + 1) / 2 ? 2 : 3);
                String dcName = dcNames.getOrDefault(dcId, "Datacenter #" + dcId);
                Log.printLine("VM #" + vm.getId() + " (MIPS: " + vm.getMips() + ") -> Datacenter: " + dcName + " (ID: " + dcId + ")");
            }
        }

        Log.printLine("\n========== DATACENTER JOB EXECUTION BREAKDOWN ==========");
        for (Map.Entry<Integer, Integer> entry : dcJobCount.entrySet()) {
            int dcId = entry.getKey();
            String dcName = dcNames.getOrDefault(dcId, "Datacenter #" + dcId);
            Log.printLine("Datacenter " + dcName + " (ID: " + dcId + ") processed " + entry.getValue() + " jobs ("
                    + String.format("%.1f%%", 100.0 * entry.getValue() / list.size()) + ")");
        }
    }

    private static WorkflowDatacenter createDatacenter(String name, int vmCapacity) throws Exception {
        List<Host> hostList = new ArrayList<>();
        List<Pe> peList = new ArrayList<>();

        int mips = 3000;
        int peCount = vmCapacity;
        for (int i = 0; i < peCount; i++) {
            peList.add(new Pe(i, new PeProvisionerSimple(mips)));
        }

        int hostId = 0;
        int ram = 512 * vmCapacity; // host RAM: exactly fits vmCapacity VMs of 512 MB
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
        ClusterStorage s1 = new ClusterStorage(name, 1e12);
        s1.setMaxTransferRate(Integer.MAX_VALUE);
        s1.setBandwidth("local", 3.0e7);
        s1.setBandwidth("source", 3.0e7);
        s1.setBandwidth("Datacenter_0", 3.0e7);
        s1.setBandwidth("Datacenter_1", 3.0e7);
        storageList.add(s1);
        DatacenterCharacteristics characteristics = new DatacenterCharacteristics(
                arch, os, vmm, hostList, timeZone, cost, costPerMem, costPerStorage, costPerBw);

        return new WorkflowDatacenter(
                name, characteristics, new VmAllocationPolicySimple(hostList), storageList, 0);
    }
}