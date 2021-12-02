package se.oru.assignment.assignment_oru.test;

import java.io.File;

import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import com.google.ortools.Loader;
import com.google.ortools.linearsolver.MPSolver;
import com.vividsolutions.jts.geom.Coordinate;

import se.oru.assignment.assignment_oru.OptimizationProblem;
import se.oru.assignment.assignment_oru.OptimizationProblem.OptimizationSolverType;
import se.oru.assignment.assignment_oru.Task;
import se.oru.assignment.assignment_oru.methods.SystematicAlgorithm;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.NetworkConfiguration;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;

import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TimedTrajectoryEnvelopeCoordinatorSimulation;

import se.oru.coordination.coordination_oru.util.BrowserVisualization;

import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.Robot;




@DemoDescription(desc = "One-shot navigation of 3 robots coordinating on paths obtained with the ReedsSheppCarPlanner.")
public class TaskAssignmentMultiRobotsWithoutMap {
	//load library used for optimization
	 static {
		    //System.loadLibrary("jniortools");
		    Loader.loadNativeLibraries();
	}
	public static void main(String[] args) throws InterruptedException {
		//Max Vel and Acc for the robots
		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		//Instantiate a timed trajectory envelope coordinator.
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		//final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		final TimedTrajectoryEnvelopeCoordinatorSimulation tec = new TimedTrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				CriticalSection cs = o1.getCriticalSection();
				RobotReport robotReport1 = o1.getRobotReport();
				RobotReport robotReport2 = o2.getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		});
		
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				return (o2.getRobotReport().getRobotID()-o1.getRobotReport().getRobotID());
			}
		});
		
		tec.setBreakDeadlocks(false, true, true);
		
		NetworkConfiguration.setDelays(0,0);
		NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS = 0.;
		tec.setNetworkParameters(NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS, NetworkConfiguration.getMaximumTxDelay(), 0.01);

		
		
		
		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		
		
		Coordinate[] footprint = new Coordinate[] { 
				footprint1,
				footprint2,
				footprint3,
				footprint4,
		};
		
		
		
		
		
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.setForwardModel(3, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.setForwardModel(4, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.setForwardModel(5, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		
		//Need to instantiate the fleetmaster interface
		tec.instantiateFleetMaster(0.1, false);
		
		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		
		
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();
		
		
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//viz.setSize(1024, 768);
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(20, 44.46, 17.26);
		tec.setVisualization(viz);
		tec.setUseInternalCriticalPoints(false);


		
	   
		Pose startPoseRobot1 = new Pose(15.0,13.0,Math.PI/2);
		Pose startPoseRobot2 = new Pose(18.0,13.0,Math.PI/2);
		Pose startPoseRobot3 = new Pose(21.0,13.0,Math.PI/2);
		Pose startPoseRobot4 = new Pose(24.0,13.0,Math.PI/2);
		Pose startPoseRobot5 = new Pose(27.0,13.0,Math.PI/2);

		tec.setFootprint(1, footprint);
		tec.setFootprint(2, footprint);
		tec.setFootprint(3, footprint);
		tec.setFootprint(4, footprint);
		tec.setFootprint(5, footprint);

		
		
		Robot robot1 = new Robot(1, 1);
		Robot robot2 = new Robot(2, 1);
		Robot robot3 = new Robot(3, 1);
		Robot robot4 = new Robot(4, 1);
		Robot robot5 = new Robot(5, 1);
		
		
		
		tec.addRobot(robot1,startPoseRobot1);
		tec.addRobot(robot2,startPoseRobot2);
		tec.addRobot(robot3,startPoseRobot3);
		tec.addRobot(robot4, startPoseRobot4);
		tec.addRobot(robot5, startPoseRobot5);
		
		String yamlFile = "maps/map-empty.yaml";
		
		
		
		

		Pose startPoseGoal1 = new Pose(16.0,25.0,Math.PI/2);
		Pose startPoseGoal2 = new Pose(19.0,25.0,Math.PI/2);
		Pose startPoseGoal3 = new Pose(22.0,25.0,Math.PI/2);
		Pose startPoseGoal4 = new Pose(25.0,25.0,Math.PI/2);
		Pose startPoseGoal5 = new Pose(28.0,25.0,Math.PI/2);
		
		Pose startPoseGoal6 = new Pose(4.0,9.0,Math.PI/2);
		
		
		Pose goalPoseGoal1 = new Pose(16.0,35.0,Math.PI/2);
		Pose goalPoseGoal2 = new Pose(19.0,35.0,Math.PI/2);
		Pose goalPoseGoal3 = new Pose(22.0,35.0,Math.PI/2);
		Pose goalPoseGoal4 = new Pose(25.0,35.0,Math.PI/2);
		Pose goalPoseGoal5 = new Pose(28.0,35.0,Math.PI/2);
		
		Pose goalPoseGoal6 = new Pose(4.0,30.0,Math.PI/2);
		
		
	

		

		
		
		//Pose startPoseGoal4 = new Pose(27.0,20.0,Math.PI/2);
		//Pose startPoseGoal5 = new Pose(26.0,5.0,0.0);
		
	
		//Pose goalPoseRobot4 = new Pose(48.0,27.0,0.0);
		//Pose goalPoseRobot5 = new Pose(52.0,6.0,0.0);

		Task task1 = new Task(1,startPoseGoal1,goalPoseGoal1,1);
		Task task2 = new Task(2,startPoseGoal2,goalPoseGoal2,1);
		Task task3 = new Task(3,startPoseGoal3,goalPoseGoal3,1);

		Task task4 = new Task(4,startPoseGoal4,goalPoseGoal4,1);
		Task task5 = new Task(5,startPoseGoal5,goalPoseGoal5,1);
		
		//Task task6 = new Task(6,startPoseGoal6,goalPoseGoal6,1);
	
		//TaskAssignmentSimulatedAnnealing assignmentProblem = new TaskAssignmentSimulatedAnnealing();
		OptimizationProblem assignmentProblem = new OptimizationProblem();
		int numPaths = 1;
		assignmentProblem.addTask(task1);
		assignmentProblem.addTask(task2);
		assignmentProblem.addTask(task3);
		//assignmentProblem.addTask(task4);
		//assignmentProblem.addTask(task5);
		//assignmentProblem.addTask(task6);
		
		for (int robotID : tec.getIdleRobots()) {
				footprint = tec.getFootprint(robotID);
				//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
				ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
				rsp.setRadius(0.2);
				rsp.setFootprint(footprint);
				rsp.setTurningRadius(4.0);
				rsp.setDistanceBetweenPathPoints(0.1);
				//rsp.setMap(yamlFile);
				double res = 0.2;// Double.parseDouble(getProperty("resolution", yamlFile));
				//rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", "maps/CentroPiaggio.yaml"));
				//double res = Double.parseDouble(Missions.getProperty("resolution", "maps/CentroPiaggio.yaml"));
				//rsp.setMapResolution(res);
				rsp.setPlanningTimeInSecs(2);
				tec.setMotionPlanner(robotID, rsp);
				tec.setNominalTrajectoryParameters(robotID, MAX_VEL, MAX_VEL, false, -1, -1, -1, MAX_ACCEL, -1, -1);
				
				
		}
		
	    ///////////////////////////////////////////////////////
		
		double [][][]optimalAllocation = {{{0.0},{0.0},{1.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{1.0},{0.0}},
				{{0.0},{1.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{1.0}},
				{{1.0},{0.0},{0.0},{0.0},{0.0}}};
		
		//Solve the problem to find some feasible solution
		double alpha = 1.0;
		
		//tec.setFakeCoordinator(true);
		//tec.setAvoidDeadlocksGlobally(true);
		//assignmentProblem.LoadScenario("ProvaScenario");
		//assignmentProblem.LoadScenarioAllocation(optimalAllocation);
		
		
		assignmentProblem.setmaxNumberOfAlternativePaths(numPaths);
		assignmentProblem.setminMaxVelandAccel(MAX_VEL, MAX_ACCEL);
		assignmentProblem.setCoordinator(tec);
		assignmentProblem.instantiateFleetMaster(0.1, false);
		
		
		
		assignmentProblem.setFleetVisualization(viz);
		
		assignmentProblem.setLinearWeight(alpha);
		assignmentProblem.setCostFunctionsWeight(1.0, 0.0, 0.0);
	
		
		MPSolver solver = assignmentProblem.createOptimizationProblem(tec);
		
		
		double [][][] assignmentMatrix = assignmentProblem.solveOptimizationProblem(solver,tec, OptimizationSolverType.SYSTEMATIC_ALGORITHM);
		//double [][][] assignmentMatrix = assignmentProblem.solveOptimizationProblemLocalSearch(tec,-1);
		//assignmentProblem.LoadScenarioAllocation(optimalAllocation);
		for (int i = 0; i < assignmentMatrix.length; i++) {
			for (int j = 0; j < assignmentMatrix[0].length; j++) {
				for(int s = 0; s < numPaths; s++) {
					//System.out.println("x"+"["+(i+1)+","+(j+1)+","+(s+1)+"]"+" is "+ optimalAllocation[i][j][s]);
					if(assignmentMatrix[i][j][s] == 1) {
						//System.out.println("Robot " +(i+1) +" is assigned to Task "+ (j+1)+" throw Path " + (s+1));
					}
				}
			} 
		}
		assignmentProblem.allocateTaskstoRobots(assignmentMatrix,tec);	
	}
}
