package org.workflowsim.examples;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.cloudbus.cloudsim.Cloudlet;
import org.cloudbus.cloudsim.Log;
import org.cloudbus.cloudsim.core.CloudSim;
import org.workflowsim.CondorVM;
import org.workflowsim.WorkflowDatacenter;
import org.workflowsim.WorkflowEngine;
import org.workflowsim.WorkflowPlanner;
import org.workflowsim.utils.Parameters;

/**
 * Driver for HICA planning algorithm using shared SimSetup.
 */
public class HICA {

    public static void main(String[] args) {
        try {
            int vmNum = 20;
            String daxPath = "config/dax/Montage_50.xml";

            if (!SimSetup.setupParameters(vmNum, daxPath, Parameters.PlanningAlgorithm.HICA)) {
                return;
            }

            WorkflowDatacenter datacenter0 = SimSetup.createDatacenter("Datacenter_0", (vmNum + 1) / 2);
            WorkflowDatacenter datacenter1 = SimSetup.createDatacenter("Datacenter_1", vmNum / 2);

            WorkflowPlanner wfPlanner = new WorkflowPlanner("planner_0", 1);
            WorkflowEngine wfEngine = wfPlanner.getWorkflowEngine();

            List<CondorVM> vmList = SimSetup.createVM(wfEngine.getSchedulerId(0), vmNum);
            wfEngine.submitVmList(vmList, 0);
            wfEngine.bindSchedulerDatacenter(datacenter0.getId(), 0);
            wfEngine.bindSchedulerDatacenter(datacenter1.getId(), 0);

            Map<Integer, String> dcNames = new HashMap<>();
            dcNames.put(datacenter0.getId(), datacenter0.getName());
            dcNames.put(datacenter1.getId(), datacenter1.getName());

            CloudSim.startSimulation();
            List<Cloudlet> outputList = wfEngine.getJobsReceivedList();
            CloudSim.stopSimulation();

            SimSetup.printResults("HICA RESULTS", outputList, vmList, dcNames);

        } catch (Exception e) {
            Log.printLine("Error: " + e.getMessage());
            e.printStackTrace();
        }
    }
}