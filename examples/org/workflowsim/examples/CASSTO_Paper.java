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


public class CASSTO_Paper {

    public static void main(String[] args) {
        try {
            int vmNum = 20;
            String daxPath = System.getProperty("dax", "config/dax/Montage_100.xml");

            // Requires a new enum constant Parameters.PlanningAlgorithm.CASSTO_PAPER,
            // registered next to CASSTO so it maps to CASSTOPaperPlanningAlgorithm.
            if (!SimSetup.setupParameters(vmNum, daxPath, Parameters.PlanningAlgorithm.CASSTO_PAPER)) {
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

            SimSetup.printResults("CASSTO-PAPER RESULTS", outputList, vmList, dcNames,
                    org.workflowsim.planning.CASSTOPaperPlanningAlgorithm.bestEstimatedMakespan);

        } catch (Exception e) {
            Log.printLine("Error: " + e.getMessage());
            e.printStackTrace();
        }
    }
}