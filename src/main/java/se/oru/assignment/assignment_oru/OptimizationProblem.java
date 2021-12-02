package se.oru.assignment.assignment_oru;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.TreeSet;
import java.util.logging.Logger;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;
import org.metacsp.utility.UI.Callback;
import org.metacsp.utility.logging.MetaCSPLogging;


import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.Polygon;
import aima.core.util.datastructure.Pair;
import se.oru.assignment.assignment_oru.methods.SystematicAlgorithm;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.IndexedDelay;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.fleetmasterinterface.AbstractFleetMasterInterface;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterface;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.CumulatedIndexedDelaysList;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.util.FleetVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import com.google.ortools.linearsolver.*;



public class OptimizationProblem {
	 
		//Optimization Problem Parameters
		protected int numRobot;
		protected int numTask;
		protected int dummyRobot;
		protected int dummyTask;
		protected int numRobotAug;
		protected int numTaskAug;
		protected int maxNumPaths = 1;
		protected double linearWeight = 1;
		protected double [][][] costValuesMatrix;
		protected ArrayList <Task> taskQueue = new ArrayList <Task>();
		protected ArrayList <Task> taskPosponedQueue = new ArrayList <Task>();
		//Parameters of weights in Optimization Problem
		protected double pathLengthWeight = 1;
		protected double arrivalTimeWeight = 0;
		protected double tardinessWeight = 0;
		protected double timeOut = Double.POSITIVE_INFINITY;
		protected int numIteration = -1;
		
		//Number of Idle Robots (i.e. robot that are free to execute a task)
		protected ArrayList <Integer> IDsIdleRobots = new ArrayList <Integer>();
		protected ArrayList <Integer> realTasksIDs = new ArrayList <Integer>();
		protected ArrayList <Integer> robotsIDs = new ArrayList <Integer>(); //this is the set of IDs of all the robots considered into the problem (i.e. both real and virtual robots)
		protected ArrayList <Integer> tasksIDs = new ArrayList <Integer>(); //this is the set of IDs of all the tasks considered into the problem (i.e. both real and virtual tasks)
		
		protected int virtualRobotID = Integer.MAX_VALUE; 
		protected int virtualTaskID = Integer.MAX_VALUE;
		
		
		//ROADMAP Parameters
		protected String scenario;
		protected double [][][] ScenarioAllocation;
		protected int numAllocation = 1;
		protected boolean saveFutureAllocations = false;
		protected int previousAssignment = 0;
		
		//Path and arrival Time Parameters
		//Infinity cost if path to reach a goal note exists
		protected double MaxPathLength = 10000000; // a finite value is assumed to avoid problem with  orTool
		//This is the sum of max path length for each robot
		protected double sumMaxPathsLength = 1;
		//This is the sum of arrival time considering max path length for each robot
		protected double sumArrivalTime = 1;
		//Parameters of mininum Velocity and Acceleration considering all robots
		protected double minMaxVel;
		protected double minMaxAcc;
		//This is the sum of all tardiness 
		protected double sumTardiness = 1;
		
		//Motion planner and Coordinator Parameters
		protected AbstractTrajectoryEnvelopeCoordinator coordinator;
		
		

		
		//Time required by function parameters
		protected long timeRequiretoEvaluatePaths;
		protected long timeRequiretofillInPall;
		protected long timeRequiretoComputeCriticalSection;
		protected long timeRequiretoComputePathsDelay;
		protected long initialTime;
		
		protected HashMap<Integer, PoseSteering[]> pathsToTargetGoal =  new HashMap<Integer, PoseSteering[]>();
		protected ArrayList <SpatialEnvelope> pathsDrivingRobot = new ArrayList <SpatialEnvelope>();
		protected HashMap<Integer,CriticalSection [][][][]> criticalSections =  new HashMap<Integer, CriticalSection [][][][]>();
	
		//FleetMaster Interface Parameters
		
		protected AbstractFleetMasterInterface fleetMasterInterface = null;
		protected boolean propagateDelays = false;
		protected static Logger metaCSPLogger = MetaCSPLogging.getLogger(TrajectoryEnvelopeCoordinator.class);
		
		
		
		
		//Task Allocation Thread Parameters 
		protected int CONTROL_PERIOD_Task = 20000;
		public static int EFFECTIVE_CONTROL_PERIOD_task = 0;
		protected FleetVisualization viz = null;
		
		
		
		 public enum OptimizationSolverType {
			 SYSTEMATIC_ALGORITHM,
			 GRADIENT_DESCENT_ALGORITHM,
			 SIMULATED_ANNEALING_ALGORITHM,
			 GREEDY_ALGORITHM,
			 EXACT_ALGORITHM,
			 
		 }
		
		/**
		 * Save the scenario(Missions + paths) for all the optimization problems that are solved ( if some tasks are assigned online)
		 * 
		 */
		public void saveAllScenarios() {
			this.saveFutureAllocations = true;
			
		}
		
		
		
		
		/**
		 * Set the timeOut for the optimization problem in minutes. The algorithm will search a solution until this time.
		 * Use number from 0.1 (10 seconds) to 0.6 (60 seconds) for seconds
		 * @param minutes -> timeout value in minutes
		 * 
		 */
		public void setTimeOutinMinutes(double minutes) {
			if(timeOut < 0) {
				throw new Error("Timeout cannot be negative!");
			}
			if(timeOut < 0.6) {
				this.timeOut = timeOut*1000;
			}
			else {
				this.timeOut = timeOut*60*1000;
			}
			
		}
	
		
		/**
		 * Set the maximum alternative number of paths to reach a goal for each robot.
		 * @param alternativePaths -> number of path to reach a goal
		 */
		public void setmaxNumberOfAlternativePaths(int alternativePaths) {
			
			this.maxNumPaths = alternativePaths;
		}
		
		
		/** Get the maximum alternative number of paths to reach a goal for each robot.
		 */
		public int getmaxNumberOfAlternativePaths() {
			
			return this.maxNumPaths;
		}
		
		
		public double getSumArrivalTime() {
			return this.sumArrivalTime;
		}
		
		
		public double getTimeout() {
			return this.timeOut;
		}
		
		public double [][][] getCostValuesMatrix() {
			return this.costValuesMatrix;
		}
		
		
		
		public void saveMatrixinFile(String filename, double[][][] optimalAssignmentMatrix) {
		    try {
		        BufferedWriter bw = new BufferedWriter(new FileWriter((filename),true));
		        bw.write("{{");
		        for (int i = 0; i < optimalAssignmentMatrix.length; i++) {
		        	for (int j = 0; j < optimalAssignmentMatrix[i].length; j++) {
		        		bw.write("{");
		        		for (int s = 0; s < maxNumPaths; s++) {
		        			bw.write(optimalAssignmentMatrix[i][j][s]+"");
		        		}
		        		bw.write("}");
		        		bw.write(",");
		        		
		        	}
		        	bw.write("}");
		        	bw.write(",");
		            bw.newLine();
		            bw.write("{");
		        }
		        bw.write("-------------");
		        bw.newLine();
		        bw.flush();
		        bw.close();
		    } catch (IOException e) {}
		}
		
		
		/**
		 * Load a Scenario 
		 * @param scenario ->  Scenario to load
		 */
		
		public void loadScenario(String scenario) {
			this.scenario = scenario;
		}
		
		/**
		 * Load an Optimal Task Allocation
		 * @param scenario ->  Scenario to load
		 */
		
		public void loadOptimalAllocation(double [][][] Allocation) {
			this.ScenarioAllocation = Allocation;
		}
		
		/**
		 * Set the Coordinator 
		 * @param viz -> Visualization to use 
		 */
		
		public void setCoordinator(AbstractTrajectoryEnvelopeCoordinator coordinator) {
			this.coordinator = coordinator;
		}
		
		
		/**
		 * Set the weights of cost functions considered in function B (i.e. all costs related to the single robot)in Optimization Problem. These must be numbers between 0 and 1.
		 * Three costs functions are considered into B: path length, nominal arrival time, tardiness. This function allow to set the weight (i.e. the importance) to each of then
		 * Default values are (1,0,0)
		 * @param pathLengthWeight ->  The path length weight;
		 * @param arrivalTimeWeight -> The arrival time weight;
		 * @param tardinessWeight -> The tardiness weight
		 */
		
		public void setCostFunctionsWeight(double pathLengthWeight,double arrivalTimeWeight,double tardinessWeight) {
			if(pathLengthWeight <0 || arrivalTimeWeight < 0 ||  tardinessWeight < 0) {
				throw new Error("Weights cannot be  numbers less than 0!");
			}
			double sumWeight = pathLengthWeight + arrivalTimeWeight + tardinessWeight;
			if(sumWeight != 1) {
				throw new Error("The sum of weights must be equal to 1!");
			}
			this.pathLengthWeight = pathLengthWeight;
			this.arrivalTimeWeight = arrivalTimeWeight;
			this.tardinessWeight = tardinessWeight;
		}
		
		
		
		/**
		 * Set the linear weight used in Optimization Problem. This parameter sets the weight (i.e. the importance) of B and F. If alpha = 1 (default value) only the B function is minimized.
		 * linearWeight must be > 0 and < 1. 
		 * More this value is close to 0, more is the importance given to F Function. More this value is close to 1, more is the importance given to B function.
		 * @param alpha -> the linear weight value. This value is used to weight differently the B and the F function
		 */
		
		public void setLinearWeight(double alpha) {
			if(alpha < 0 || alpha > 1) {
				throw new Error("The linear weigth must be > 0 and < 1!");
			}
			metaCSPLogger.info("[OptimizationProblem] alpha is set to : " + alpha);
			this.linearWeight = alpha;
		}	
		
		
		/**
		 * Get the linear weight used in Optimization Problem.This parameter sets the weight (i.e. the importance) of B and F. If alpha = 1 (default value) only the B function is minimized.
		 * @return the value of the linear weight alpha
		 */
		
		public double getLinearWeight() {
			return this.linearWeight;
		}	
		
		
	/**
	 * Set the Fleet Visualization 
	 * @param viz -> Visualization to use 
	 */
			
	public void setFleetVisualization(FleetVisualization viz) {
		this.viz = viz;
	}
	
	/**
	 * Initialize the IDs of all the robots considered into the problem (both real and virtual)
	 */
	
	private void initializeRobotsIDs() {
		int virtualRobotID = this.virtualRobotID;
		for(int i= 0; i < numRobotAug; i++) {
			if(i < IDsIdleRobots.size()) {
				robotsIDs.add(IDsIdleRobots.get(i));	
			}else {
				robotsIDs.add(virtualRobotID);
				virtualRobotID = virtualRobotID-1;
				this.virtualRobotID -=1;
			}	
		}
	}
	/**
	 * Get the IDs of all the robots considered into the problem (both real and virtual)
	 * @return a set of all the robots IDs
	 */
	
	public ArrayList <Integer> getRobotsIDs() {
		return this.robotsIDs;
	}
	
	
	
	/**
	 * Initialize the IDs of all the tasks considered into the problem (both real and virtual)
	 */
	
	protected void initializeTasksIDs() {
		int virtaulTaskID = this.virtualTaskID;
		for(int i= 0; i < numTaskAug; i++) {
			if(i < taskQueue.size()) {
				realTasksIDs.add(taskQueue.get(i).getID());
				tasksIDs.add(taskQueue.get(i).getID());
			}else {
				tasksIDs.add(virtaulTaskID);
				virtaulTaskID = virtaulTaskID-1;
				this.virtualTaskID -=1;
			}	
		}
	}
	
	/**
	 * Get the IDs of all the tasks considered into the problem (both real and virtual)
	 */
	
	public ArrayList <Integer> getTasksIDs() {
		return this.tasksIDs;
	}
	
	/**
	 * Check if a goal can be reached by at least one robot of the Fleet . In this case a dummy robot and task is added for each unreachable location.
	 * @param PAll -> the 3D matrix of all paths 
	 * @return An updated PAll incremented by 1 on each direction (i.e. i, j, p_ij)
	 */
	
	private  double [][][] checkTargetGoals (double [][][] PAll){
		for (int j= 0; j< PAll[0].length ; j++) {
			boolean targetEndCanBeReach = false;
			for (int i = 0; i < PAll.length; i++) {
				for (int s = 0; s < maxNumPaths; s++) {
					if(PAll[i][j][s] != MaxPathLength) {
						targetEndCanBeReach = true;
					}
				}
			}
			//no robot can reach the target end -> need to introduce a dummy task and robot 
			if(!targetEndCanBeReach) {
				dummyRobot += 1 ;
				dummyTask += 1 ;
				numRobotAug += 1;
				numTaskAug += 1;
	
			}	
		}
		
		double [][][] PAllAug = new double [numRobotAug][numTaskAug][maxNumPaths];
		int robotID = 0;
		int taskID = 0;
		for(int i = 0;i < numRobotAug; i++) {
			for(int j = 0; j<  numTaskAug;j++) {
				for (int s = 0; s < maxNumPaths; s++) {
					//copy the values from Pall
					if(i < PAll.length && j< PAll[0].length) {
						PAllAug[i][j][s] = PAll[i][j][s];
					}else {
						if(i < robotsIDs.size()) {
							robotID = robotsIDs.get(i);						
						}else {
							robotID = this.virtualRobotID -1 ;
							robotsIDs.add(robotID);
						}
						if (j < tasksIDs.size()) {
							taskID = tasksIDs.get(j);
						}else {
							taskID = virtualTaskID - 1;
							this.virtualTaskID -= 1;
							tasksIDs.add(taskID);
						}
						PAllAug[i][j][s] = 1;
						pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+s, null); //FIXME null may be not correct
				
					}
				}
					
			}
		}
		return PAllAug;
	}
	
	

	/**
	 * Set the minimum values of maximum velocity and acceleration considering all robots of the fleet
	 * @param MaxVel -> minimum of maximum velocity of all robot models;
	 * @param MaxAccel -> minimum of maximum acceleration of all robot models;
	 */
	
	public void setminMaxVelandAccel(double MaxVel,double MaxAccel) {
		this.minMaxVel = MaxVel;
		this.minMaxAcc = MaxAccel;
	}
	
	
	/**
	 * Sort the set of tasks task by deadlines
	 * @author Paolo Forte
	 *
	 */
	
	class SortByDeadline implements Comparator<Task>{
		@Override
		public int compare(Task task1, Task task2) {
			return (int) (task1.getDeadline()-task2.getDeadline());
		}
	}
	
	
	/**
	 * Sort the set of tasks by deadlines. Tasks with the closest deadline are prioritized.
	*/
	
	private void checkOnTaskDeadline() {
		ArrayList <Task> sortedtaskArray = new ArrayList <Task>();
		for(int j=0; j < taskQueue.size(); j++ ) {
			sortedtaskArray.add(taskQueue.get(j));
		}
		sortedtaskArray.sort(new SortByDeadline());
		for(int i=0;i < IDsIdleRobots.size(); i++) {
			int index = taskQueue.indexOf(sortedtaskArray.get(i));
			if(taskQueue.get(index).getDeadline() != -1) {
				taskQueue.get(index).setPriority(true);
			}
			
		}
	}
	
	/**
	 * Delete one or multiple tasks from taskQueue in order to avoid blocking 
	*/
	
	private void checkOnBlocking() { 
		int removedTask = 0;
		Coordinate[] taskFootprint = coordinator.getMaxFootprint();
		for(int j=0; j < taskQueue.size(); j++ ) {
			Task currentTask = taskQueue.get(j);
			double xTask=currentTask.getGoalPose().getX();
			double yTask=currentTask.getGoalPose().getY();
			double dist1 = taskFootprint[0].distance(taskFootprint[1])/2;
			double dist2 = taskFootprint[1].distance(taskFootprint[2])/2;
			Coordinate Taskfootprint1 = new Coordinate((xTask-dist1),(yTask+dist2));
			Coordinate Taskfootprint2 = new Coordinate((xTask+dist1),(yTask+dist2));
			Coordinate Taskfootprint3 = new Coordinate((xTask+dist1),(yTask-dist2));
			Coordinate Taskfootprint4 = new Coordinate((xTask-dist1),(yTask-dist2));
			Polygon ll=TrajectoryEnvelope.createFootprintPolygon(Taskfootprint1,Taskfootprint2,Taskfootprint3,Taskfootprint4);
			
			for(int k = 0; k < taskQueue.size(); k++ ) {
				if(k != j) {
					Task taskProva2 = taskQueue.get(k);
					double xTask2=taskProva2.getGoalPose().getX();
					double yTask2=taskProva2.getGoalPose().getY();
					Coordinate Taskfootprint5 = new Coordinate((xTask2-dist1),(yTask2+dist1));
					Coordinate Taskfootprint6 = new Coordinate((xTask2+dist1),(yTask2+dist2));
					Coordinate Taskfootprint7 = new Coordinate((xTask2+dist1),(yTask2-dist2));
					Coordinate Taskfootprint8 = new Coordinate((xTask2-dist1),(yTask2-dist2));
					Polygon gg = TrajectoryEnvelope.createFootprintPolygon(Taskfootprint5,Taskfootprint6,Taskfootprint7,Taskfootprint8);
					if(ll.intersects(gg) ) {
						if(taskProva2.getDeadline() == -1 && currentTask.getDeadline() == -1) {
							taskQueue.remove(k);
							taskPosponedQueue.add(taskProva2);
							removedTask += 1;
						}else {
							if (taskProva2.getDeadline() == -1 && currentTask.getDeadline() != -1){
								taskQueue.remove(k);
								taskPosponedQueue.add(taskProva2);
								removedTask += 1;

							}else if(taskProva2.getDeadline() != -1 && currentTask.getDeadline() == -1) {
								taskQueue.remove(j);
								taskPosponedQueue.add(currentTask);
								removedTask += 1;
								j -=1;
								break;
							}else {
								if(taskProva2.getDeadline() > currentTask.getDeadline() ) {
									taskQueue.remove(k);
									taskPosponedQueue.add(taskProva2);
									removedTask += 1;
								}else {
									taskQueue.remove(j);
									taskPosponedQueue.add(currentTask);
									removedTask += 1;
									j -=1;
									break;
								}	
							}
						}
						
					}
					
				}
			}
		} //end for loop
		metaCSPLogger.info("[OptimizationProblem] Task posponed for avoid blocking : " + removedTask);
	}
	
	/**
	 * Enable and initialize the fleetmaster library to estimate precedences to minimize the overall completion time.
	 * Note: this function should be called before placing the first robot.
	 * ATTENTION: If dynamic_size is <code>false</code>, then the user should check that all the paths will lay in the given area.
	 * @param origin_x The x coordinate (in meters and in global inertial frame) of the lower-left pixel of fleetmaster GridMap.
	 * @param origin_y The y coordinate (in meters and in global inertial frame) of the lower-left pixel of fleetmaster GridMap.
	 * @param origin_theta The theta coordinate (in rads) of the lower-left pixel map (counterclockwise rotation). Many parts of the system currently ignore it.
	 * @param resolution The resolution of the map (in meters/cell), 0.01 <= resolution <= 1. It is assumed this parameter to be global among the fleet.
	 * 					 The highest the value, the less accurate the estimation, the lowest the more the computational effort.
	 * @param width Number of columns of the map (>= 1) if dynamic sizing is not enabled.
	 * @param height Number of rows of the map (>= 1) if dynamic sizing is not enabled.
	 * @param dynamic_size If <code>true</code>, it allows to store only the bounding box containing each path.
	 * @param propagateDelays If <code>true</code>, it enables the delay propagation.
	 * @param debug If <code>true</code>, it enables writing to screen debugging info.
	 */
	public void instantiateFleetMaster(double origin_x, double origin_y, double origin_theta, double resolution, long width, long height, boolean dynamic_size, boolean propagateDelays, boolean debug) {
		this.fleetMasterInterface = new FleetMasterInterface(origin_x, origin_y, origin_theta, resolution, width, height, dynamic_size, debug);
		this.fleetMasterInterface.setDefaultFootprint(coordinator.getDefaultFootprint());
		this.propagateDelays = propagateDelays;
	}
	
	/**
	 * Enable and initialize the fleetmaster library to estimate precedences to minimize the overall completion time
	 * while minimizing the computational requirements (bounding box are used to set the size of each path-image).
	 * Note: this function should be called before placing the first robot.
	 * @param resolution The resolution of the map (in meters/cell), 0.01 <= resolution <= 1. It is assumed this parameter to be global among the fleet.
	 * 					 The highest the value, the less accurate the estimation, the lowest the more the computational effort.
	 * @param propagateDelays If <code>true</code>, it enables the delay propagation.
	 */
	public void instantiateFleetMaster(double resolution, boolean propagateDelays) {
		this.fleetMasterInterface = new FleetMasterInterface(0., 0., 0., resolution, 100, 100, true, false);
		this.fleetMasterInterface.setDefaultFootprint(coordinator.getDefaultFootprint());
		this.propagateDelays = propagateDelays;
	}
	
	
	/**
	 * Add a path to the fleetmaster interface
	 * @param robotID -> The ID of the robot
	 * @param pathID -> the ID of the path
	 * @param pss -> the path expressed as a PoseSteering vector
	 * @param boundingBox -> the bounding box of the path
	 * @param coordinates -> footprint of the robot 
	 */
	protected void addPath(int robotID, int pathID, PoseSteering[] pss, Geometry boundingBox, Coordinate... coordinates) {
		if (!fleetMasterInterface.addPath(robotID, pathID, pss, boundingBox, coordinates)) 
			metaCSPLogger.severe("Unable to add the path to the fleetmaster gridmap. Check if the map contains the given path.");
	}
	
	
	
	/**
	 * Delete the path from the fleetmaster interface
	 * @param pathID -> The ID of the path to remove 
	 */
	protected void removePath(int pathID){
		if (!fleetMasterInterface.clearPath(pathID)) 
			metaCSPLogger.severe("Unable to remove the path to the fleetmaster gridmap. Check if the map contains the given path.");
	}
	
	
	protected CumulatedIndexedDelaysList toIndexedDelaysList(TreeSet<IndexedDelay> delays, int max_depth) {
		//Handle exceptions
		if (delays == null) {
			metaCSPLogger.severe("Invalid input in function toPropagationTCDelays!!");
			throw new Error("Invalid input in function toPropagationTCDelays!!");
		}
		if (delays.isEmpty() || max_depth < 1) return new CumulatedIndexedDelaysList();
			
		//Cast the type
		ArrayList<Long> indices = new ArrayList<Long>();
		ArrayList<Double> values = new ArrayList<Double>();
		Iterator<IndexedDelay> it = delays.descendingIterator();
		IndexedDelay prev = delays.last();
		while (it.hasNext()) {
			IndexedDelay current = it.next();
			//Check unfeasible values
			if (current.getValue() == Double.NaN) {
				metaCSPLogger.severe("NaN input in function toPropagationTCDelays!!");
				throw new Error("NaN input in function toPropagationTCDelays!!");
			}
			if (current.getValue() == Double.NEGATIVE_INFINITY) {
				metaCSPLogger.severe("-Inf input in function toPropagationTCDelays!!");
				throw new Error("-Inf input in function toPropagationTCDelays!!");
			}
			if (prev.getIndex() < current.getIndex()) {
				metaCSPLogger.severe("Invalid IndexedDelays TreeSet!!");
				throw new Error("Invalid IndexedDelays TreeSet!!");
			}
			
			//Update the value only if positive and only if the index is lower than the max depth
			if (current.getValue() > 0 && current.getValue() < Double.MAX_VALUE && current.getIndex() < max_depth) {
				if (values.size() == 0) {
					//Add the index the first time its value is positive
					indices.add(new Long(current.getIndex()));
					values.add(current.getValue());
				}
				else if (prev.getIndex() == current.getIndex())				
					//Handle multiple delays in the same critical point
					values.set(values.size()-1, values.get(values.size()-1) + current.getValue());
				else {
					//Add the cumulative value if it is not the first.
					indices.add(new Long(current.getIndex()));
					values.add(values.get(values.size()-1) + current.getValue());
				}
			}
			prev = current;
		}
		CumulatedIndexedDelaysList propTCDelays = new CumulatedIndexedDelaysList();
		if (indices.size() > 0) {
			propTCDelays.size = indices.size();
			propTCDelays.indices = ArrayUtils.toPrimitive((Long[]) indices.toArray(new Long[indices.size()]));
			ArrayUtils.reverse(propTCDelays.indices);
			propTCDelays.values = ArrayUtils.toPrimitive((Double[]) values.toArray(new Double[values.size()]));
			ArrayUtils.reverse(propTCDelays.values);
		}
		return propTCDelays;
	}
	
	
	protected Pair<Double,Double> estimateTimeToCompletionDelays(int path1ID,PoseSteering[] pss1, TreeSet<IndexedDelay> delaysRobot1, int path2ID,PoseSteering[] pss2, TreeSet<IndexedDelay> delaysRobot2, CriticalSection cs) {
		if (this.fleetMasterInterface != null && fleetMasterInterface.checkPathHasBeenAdded(path1ID)&& fleetMasterInterface.checkPathHasBeenAdded(path2ID)) {
			CumulatedIndexedDelaysList te1TCDelays = toIndexedDelaysList(delaysRobot1, pss1.length);
			metaCSPLogger.info("[estimateTimeToCompletionDelays] te1TCDelays: " + te1TCDelays.toString());
			CumulatedIndexedDelaysList te2TCDelays = toIndexedDelaysList(delaysRobot2, pss2.length);
			metaCSPLogger.info("[estimateTimeToCompletionDelays] te2TCDelays: " + te2TCDelays.toString());
			//return fleetMasterInterface.queryTimeDelay(cs, te1TCDelays, te2TCDelays);
			return fleetMasterInterface.queryTimeDelay(cs, te1TCDelays, te2TCDelays,pss1,pss2);
		}
		
		return new Pair<Double, Double> (Double.NaN, Double.NaN);
	}
	
	
	
	/**
	 * Add a Task to tasks set
	 * @param task -> the task to add
	 * @return -> true if task is added correctly, otherwise false
	 */
	public boolean addTask(Task task) {
		if (task == null) {
			metaCSPLogger.severe("No task to add. Please give a correct Task.");
			throw new Error("Cannot add the task");
		}
		boolean TaskisAdded = taskQueue.add(task);
		metaCSPLogger.info(task.toString() +  "  is added" );
		return TaskisAdded;
	}

    

	/**
	 * Compute the number of Dummy Robots and/or Tasks. Consider the possibility to have a different number of robots (N) and tasks (M). If N > M, dummy tasks are 
	 * considered, where a dummy task is a task for which a robot stay in starting position; while if M > N dummy robots
	 * are considered, where a dummy robot is only a virtual robot. 
	 * @param numRobot Number of robots
	 * @param numTasks Number of tasks
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 */
	private void dummyRobotorTask(int numRobot, int numTasks,AbstractTrajectoryEnvelopeCoordinator tec) {
		numRobotAug = numRobot;
		numTaskAug = numTasks;
		//Restore initial value for dummy robot and task
		dummyTask = 0;
		dummyRobot = 0;
		//Considering the possibility to have n != m
		//If n > m -> we have dummy robot, so at some robot is assign the task to stay in starting position
		if (numRobot > numTasks) {
			dummyTask = numRobot - numTasks;
			numTaskAug = numTasks + dummyTask;
		}
		else if (numRobot < numTasks) {
			dummyRobot = numTasks - numRobot;
			numRobotAug = numRobot + dummyRobot;
		}
		//This second check is used when we have particular cases due to Robot Type and Task Type
		//Only if we have already a dummy task this check can be avoided
		//If A robot cannot be assigned to any task 
		if (dummyTask == 0 || dummyRobot != 0) {
			for (int i = 0; i < numRobot; i++) {
				boolean flagAllocateRobot = false;
				 for (int j = 0; j < numTasks; j++) {
					 //check if robot can be assigned to one task
					 //if (taskQueue.get(j).getTaskType() == tec.getRobotType(IDsIdleRobots[i])) {
					 if (taskQueue.get(j).isCompatible(tec.getRobot(IDsIdleRobots.get(i)))) {
						 flagAllocateRobot = true;
						 
					 }
				 }
				 //the robot cannot be assigned to any task -> add a dummy robot and task
				 if (!flagAllocateRobot) {
					 dummyRobot += 1 ;
					 dummyTask += 1 ;
					 numRobotAug += 1;
					 numTaskAug += 1;
				 }
			}
		}
		//Only if we have already a dummy robot this check can be avoided
		//If A task cannot be assigned to any robot
		if (dummyRobot == 0 || dummyTask != 0) {
			for (int i = 0; i < numTasks; i++) {
				boolean flagAllocateTask = false;
				 for (int j = 0; j < numRobot; j++) {
					//check if task can be assigned to one robot
					 //if (taskQueue.get(i).getTaskType() == tec.getRobotType(IDsIdleRobots[j])) {
					 if (taskQueue.get(i).isCompatible(tec.getRobot(IDsIdleRobots.get(j)))) {
						 flagAllocateTask = true;
					 }
				 }
				 //the task cannot be assigned to any robot -> add a dummy robot and task
				 if (!flagAllocateTask) {
					 dummyRobot += 1 ;
					 dummyTask += 1 ;
					 numRobotAug += 1;
					 numTaskAug += 1;
				 }
			}
		}
	}
	
	
	/**
	 * Transform a 1D array of MPVariable into a 3D MATRIX  
	 * @param numRobot -> Number of robots
	 * @param numTasks -> Number of tasks
	 * @param optimizationProblem -> An optimization problem defined with {@link #buildOptimizationProblem}, {@link #buildOptimizationProblemWithB}  or {@link #buildOptimizationProblemWithBNormalized}
	 * @return 3D Matrix of Decision Variable of the input problem
	 */
	private MPVariable [][][] tranformArray(MPSolver optimizationProblem) {
		//Take the vector of Decision Variable from the Optimization Problem
		MPVariable [] array1D = optimizationProblem.variables();
		MPVariable [][][] decisionVariable = new MPVariable [numRobotAug][numTaskAug][maxNumPaths];
		//Store them in a 2D Matrix
	    for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTaskAug; j++) {
				 for (int s = 0; s < maxNumPaths; s++) {
					 decisionVariable[i][j][s] = array1D[i*numTaskAug*maxNumPaths+j*maxNumPaths+s];
				 }
				 
			 }
	    }
		return decisionVariable;
	}
	
	/**
	 * Impose a constraint on the optimization problem on previous optimal solution in order to not consider more it
	 * @param optimizationProblem -> An optimization problem  defined with {@link #buildOptimizationProblem},{@link #buildOptimizationProblemWithB} or {@link #buildOptimizationProblemWithBNormalized} in which a solution is found
	 * @param assignmentMatrix -> The Assignment Matrix of the actual optimal solution
	 * @return Optimization Problem updated with the new constraint on previous optimal solution found  
	 */
	public MPSolver constraintOnPreviousSolution(MPSolver optimizationProblem, double [][][] assignmentMatrix) {
		//Take decision Variable from Optimization Problem
		MPVariable [][][] DecisionVariable = tranformArray(optimizationProblem);
		//Initialize a Constraint
		//MPConstraint c2 = optimizationProblem.makeConstraint(-Double.POSITIVE_INFINITY,1);
		MPConstraint c2 = optimizationProblem.makeConstraint(0,numRobotAug-1);
		//Define the actual optimal solution as a Constraint in order to not consider more it
    	for (int i = 0; i < numRobotAug; i++) {
    		for (int j = 0; j < numTaskAug; j++) {
    			for(int s = 0;s < maxNumPaths; s++) {
    					if (assignmentMatrix[i][j][s] >0) {
    						c2.setCoefficient(DecisionVariable[i][j][s],1);
    					}
    			}
    		}		
		 }
    	//Return the updated Optimization Problem
    	return optimizationProblem;
	}

	
	/**
	 * Impose a constraint on the optimization problem on previous optimal solution cost in order to not consider more solution that has a cost higher
	 * than this. In this manner is possible to avoid some cases.
	 * @param optimizationProblem -> An optimization problem  defined with {@link #buildOptimizationProblem} in which a solution is found
	 * @param assignmentMatrix -> The Assignment Matrix of the actual optimal solution
	 * @return Optimization Problem updated with the new constraint on optimal solution cost 
	 */
	public MPSolver constraintOnCostSolution(MPSolver optimizationProblem,double objectiveValue) {
		//Take the vector of Decision Variable from the input solver
		MPVariable [][][] decisionVariable = tranformArray(optimizationProblem);
		//Add tolerance
		objectiveValue = objectiveValue + 0.0005;
		//Initialize a Constraint
		MPConstraint c3 = optimizationProblem.makeConstraint(-Double.POSITIVE_INFINITY,objectiveValue);
		//Define a constraint for which the next optimal solutions considering only B must have a cost less than objectiveValue
    	for (int i = 0; i < numRobotAug; i++) {
    		for (int j = 0; j < numTaskAug; j++) {
    			for(int s = 0;s < maxNumPaths; s++) {
    				c3.setCoefficient(decisionVariable[i][j][s],costValuesMatrix[i][j][s]);
    			}
    		}		
		 }
    	//Return the updated Optimization Problem
    	return optimizationProblem;
	}
	
	
	/**
	 * Store the solution of a optimization problem in a Matrix 
	 * @param numRobot -> Number of robots
	 * @param numTasks -> Number of tasks
	 * @param optimizationProblem -> A solved optimization problem
	 * @return Assignment matrix for the optimization problem given as input
	 */
	
	public double [][][] saveAssignmentMatrix(int numRobot,int numTasks,MPSolver optimizationProblem){
		//Take the decision variable from the optimization problem
		MPVariable [][][] decisionVariable = tranformArray(optimizationProblem);
		double [][][] assignmentMatrix = new double [numRobot][numTasks][maxNumPaths];	
		//Store decision variable values in a Matrix
		for (int i = 0; i < numRobot; i++) {
			for (int j = 0; j < numTasks; j++) {
				for(int s = 0;s < maxNumPaths; s++) {
					assignmentMatrix[i][j][s] = decisionVariable[i][j][s].solutionValue();
				}
			}
		}
		return assignmentMatrix;	
	}
	
	
	/**
	 * Evaluate the cost associated to the path length for the a couple of robot and task.
	 * If a path between a couple of robot and task does not exists the cost is consider infinity.
	 * @param robot -> The  Robot ID
	 * @param task -> The Task ID
	 * @param rsp -> The motion planner that will be called for planning for any
	 * robot. 
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 * @return The cost associated to the path length for the couple of robot and task given as input
	 */
	private synchronized double evaluatePathLength(int robotID , int taskID, int path, AbstractTrajectoryEnvelopeCoordinator tec){
		//Evaluate the path length for the actual couple of task and ID
		//Initialize the path length to infinity
		
		
		
		double pathLength = MaxPathLength;
		//take index positon of robotID in Robot set
		int robotindex = IDsIdleRobots.indexOf(robotID);
		// Only for real robots and tasks
		if (IDsIdleRobots.contains(robotID) && realTasksIDs.contains(taskID)) {
			//Take the state for the i-th Robot
			RobotReport rr = tec.getRobotReport(IDsIdleRobots.get(robotindex));
			if (rr == null) {
				metaCSPLogger.severe("RobotReport not found for Robot" + robotID + ".");
				throw new Error("RobotReport not found for Robot" + robotID + ".");
			}
			//Evaluate the path from the Robot Starting Pose to Task End Pose
			int taskIndex = realTasksIDs.indexOf(taskID);
			AbstractMotionPlanner rsp =  tec.getMotionPlanner(robotID).getCopy();
			
			rsp.setStart(rr.getPose());
			rsp.setGoals(taskQueue.get(taskIndex).getStartPose(),taskQueue.get(taskIndex).getGoalPose());
			rsp.setFootprint(tec.getFootprint(robotID));
			
			if (!rsp.plan()) {
				System.out.println("Robot" + robotID +" cannot reach the Target End of Task " + taskID);
				//the path to reach target end not exits
				pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, null);		
				//Infinity cost is returned 
				
				return pathLength;
				
			}			
			//If the path exists
			//Take the Pose Steering representing the path
			PoseSteering[] pss = rsp.getPath();
			
			System.out.println("Robot " +robotID +" taskID "+ taskID +" throw Path " + pss[pss.length-1].getX() + " " + pss[pss.length-1].getY() + " " + pss[pss.length-1].getTheta());
			
			
			//Add the path to the FleetMaster Interface -> this is necessary for F function
			addPath(robotID, pss.hashCode(), pss, null, tec.getFootprint(robotID)); 
			//SpatialEnvelope se1 = TrajectoryEnvelope.createSpatialEnvelope(pss,tec.getFootprint(robotID));
			//Geometry kk = se1.getPolygon();
			//addPath(robotID, pss.hashCode(), pss, kk, tec.getFootprint(robotID));
			
			//Save the path to Task in the path set
			pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, pss);
			//Take the Path Length
			Mission m1 = new Mission(robotID,pss);
			Missions.enqueueMission(m1);
			pathLength = Missions.getPathLength(pss);
			
		} else { //There also virtual robot and task are considered 
			//There are considered real robot and dummy task
			if (numRobot >= numTask && IDsIdleRobots.contains(robotID)){ //dummy task -> The Robot receive the task to stay in starting position
				//The second condition is used in the special case in which we have that one robot cannot be 
				//assigned to any tasks due to its type, so we must add a dummy robot and a dummy task, but we 
				//Create the task to stay in robot starting position
				PoseSteering[] dummyTask = new PoseSteering[1];
				//Take the state for the i-th Robot
				RobotReport rr = tec.getRobotReport(IDsIdleRobots.get(robotindex));
				if (rr == null) {
					metaCSPLogger.severe("RobotReport not found for Robot" + robotID + ".");
					throw new Error("RobotReport not found for Robot" + robotID + ".");
				}
				//take the starting position of the robot
				dummyTask[0] = new PoseSteering(rr.getPose(),0);
				//Add the path to the FleetMaster Interface -> so it can be considered as an obstacle from 
				//the motion planner
				
				System.out.println("Robot " +robotID +" taskID "+ taskID +" throw Path " + dummyTask[dummyTask.length-1].getX() + " " + dummyTask[dummyTask.length-1].getY() + " " + dummyTask[dummyTask.length-1].getTheta());
				
				addPath(robotID, dummyTask.hashCode(), dummyTask, null, tec.getFootprint(robotID));
				//Save the path to Dummy Task 
				pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, dummyTask);		
				//Consider a minimal pathLength
				pathLength = 1;
				Mission m1 = new Mission(robotID,dummyTask);
				Missions.enqueueMission(m1);
		
				return pathLength;
			}
			else { //There are considered dummy robot and real task
				//dummy robot -> Consider a only virtual Robot 
				PoseSteering[] dummyRobot = new PoseSteering[1];
				dummyRobot[0] = new PoseSteering(taskQueue.get(0).getGoalPose(),0);
				pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, dummyRobot);
				pathLength = 1;
				Mission m1 = new Mission(robotID,dummyRobot);
				Missions.enqueueMission(m1);
				return pathLength;
			}	
		}
		return pathLength;
	}
	
	/**
	 * Evaluate the 3D matrix Pall of all the possible combination of paths p_ij to reach a task j for a robot i, using a precomputed Scenario
	 * @return the 3D matrix Pall of all possible paths, where each element of the matrix [i] [i] [p_ij] represents the length of the path for the robot i to reach the end of task j 
	 */
	private double [][][] evaluatePAllWithScenario(){
		
		//Evaluate the path length for all the possible paths for the actual couple of task and robot
		//This cost are used then for normalizing the cost
		double sumPathsLength = 0;
		double sumArrivalTime = 0;

		Missions.loadScenario(scenario);
		
		double [][][] PAll = new double[numRobotAug][numTaskAug][maxNumPaths];
		for (int robotID : robotsIDs) {
			int robotindex = robotsIDs.indexOf(robotID);
			double maxPathLength = 1;
			for (int taskID : tasksIDs ) {
				int taskIndex = tasksIDs.indexOf(taskID);
				double pathLength = MaxPathLength;
				//Evaluate path Length
				boolean typesAreEqual = false;
				 if (taskIndex < numTask && robotindex < numRobot ) {
				 typesAreEqual = taskQueue.get(taskIndex).isCompatible(coordinator.getRobot(robotID));
				 
				 }
				 else {
					 //Considering a dummy robot or  a dummy task -> same types since there are fictitious 
					 typesAreEqual = true;
				 }
				 for(int path = 0;path < maxNumPaths; path++) {
					 if(typesAreEqual) { //try to find the mission only if robot and task are of same types  
				
						 	int missionNumber = 0;
						 	
						 	
						 	//if(IDsIdleRobots.contains(robotID)&& taskIndex < taskQueue.size()) {
						 	if(IDsIdleRobots.contains(robotID) && taskIndex < taskQueue.size()) {
						 		
						 		while(missionNumber < Missions.getMissions(robotID).size() ) {
						 		//while(Missions.getMissions(robotID).size() != 0 ) {
						 			boolean finalPositionX = ArrayUtils.isEquals(Missions.getMission(robotID, missionNumber).getToPose().getX(),taskQueue.get(taskIndex).getGoalPose().getX());
						 			boolean finalPositionY = ArrayUtils.isEquals(Missions.getMission(robotID, missionNumber).getToPose().getY(),taskQueue.get(taskIndex).getGoalPose().getY());
						 			boolean initialPositionX = ArrayUtils.isEquals(Missions.getMission(robotID, missionNumber).getFromPose().getX(),coordinator.getRobotReport(robotID).getPose().getX());
						 			boolean initialPositionY = ArrayUtils.isEquals(Missions.getMission(robotID, missionNumber).getFromPose().getY(),coordinator.getRobotReport(robotID).getPose().getY());
						 			//FIXME Missing check on steering -> it may introduce some errors 
						 			
						 			
						 			//if( ArrayUtils.isEquals(Missions.getMission(robotID, cont).getToPose().toString(),taskQueue.get(taskIndex).getGoalPose().toString())) {
						 			if( initialPositionX && initialPositionY && finalPositionX && finalPositionY) {
						 				PoseSteering[] pss = Missions.getMission(robotID, missionNumber).getPath();
							 			addPath(robotID, pss.hashCode(), pss, null, coordinator.getFootprint(robotID));
							 			pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, pss);
										pathLength = Missions.getPathLength(pss);
						
							 		 }
						 			missionNumber +=1;
							 		
							 		
							 	}
						 	}else {
						 		if (numRobot >= numTask && IDsIdleRobots.contains(robotID)){
						 			PoseSteering[] pss = new PoseSteering[] {new PoseSteering(coordinator.getRobotReport(robotID).getPose(),0)};
						 			pathLength =1 ;
						 			pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, pss);
						 			//Missions.removeMissions(Missions.getMission(robotID, cont));
						 			missionNumber += 1;
						 			
						 			
						 		}else {
						 			PoseSteering[] pss = new PoseSteering[] {new PoseSteering(taskQueue.get(0).getGoalPose(),0)};
						 			pathLength =1 ;
						 			pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, pss);
						 		}
						 	}

							
							
						
						 if ( pathLength > maxPathLength && pathLength != MaxPathLength) {
								maxPathLength = pathLength;
							}
					}else {
						 pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, null);		
					}						 
					 PAll[robotindex][taskIndex][path] = pathLength;
				}//For path
			}//For task
			
			//Sum the max path length for each robot
			sumPathsLength += maxPathLength;
			//Sum the arrival time for the max path length
			sumArrivalTime += computeArrivalTime(maxPathLength,this.minMaxVel,this.minMaxAcc);
			}
		double [][][] PAllAug =  checkTargetGoals(PAll);
		previousAssignment += numTaskAug;
		//Save the sum of max paths length to normalize path length cost
		this.sumMaxPathsLength = sumPathsLength;
		//Save the sum of arrival time considering max paths length to normalize delay cost
		this.sumArrivalTime = sumArrivalTime;
		
		//Return the cost of path length	

		return PAllAug;
		}
	
	/**
	 * Evaluate the PAll matrix, that is a matrix that contains all path for each possible combination of robot
	 * and task
	 * If a path between a couple of robot and task does not exists the cost is consider infinity.
	 * @return The PAll matrix
	 */
	/**
	 * Evaluate the PAll matrix, that is a matrix that contains all path for each possible combination of robot
	 * and task
	 * If a path between a couple of robot and task does not exists the cost is consider infinity.
	 * @param rsp -> The motion planner that will be called for planning for any
	 * robot. 
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 * @return The PAll matrix
	 */
	private double [][][] evaluatePAll(){
		
	

		
		//Evaluate the path length for the actual couple of task and ID
		//Initialize the sum of max paths lengths and time to do it for each robot
		//This cost are used then for normalizing cost
		double sumPathsLength = 0;
		double sumArrivalTime = 0;
		double [][][] PAll = new double[numRobotAug][numTaskAug][maxNumPaths];
		
		
		
		long timeInitial = Calendar.getInstance().getTimeInMillis();
		if(scenario != null) {
			double [][][] PAllScenario = evaluatePAllWithScenario();
			return PAllScenario;
		}
		for (int robotID : robotsIDs) {
			
			double maxPathLength = 1;
			int robotIndex = robotsIDs.indexOf(robotID);
			for (int taskID : tasksIDs ) {
				int taskIndex = tasksIDs.indexOf(taskID);
				
				//Evaluate path Length
				boolean typesAreEqual = false;
				 if (IDsIdleRobots.contains(robotID) && realTasksIDs.contains(taskID) ) {
				 typesAreEqual = taskQueue.get(taskIndex).isCompatible(coordinator.getRobot(robotID));
				 }
				 else {
					 //Considering a dummy robot or  a dummy task -> they don't have type
					 typesAreEqual = true;
				 }
				 for(int path = 0;path < maxNumPaths; path++) {
					 final int pathID = path;
					  if(typesAreEqual) { // only if robot and task have the same types
	
						 
						    // evaluatePathLength(robotID,taskID,pathID,tec);	
							new Thread("Robot" + robotID) {
								public void run() {
										evaluatePathLength(robotID,taskID,pathID,coordinator);								
								}		
							}.start();
							//Take time to evaluate the path
				
					}			 
					 else {
							 pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, null);		
					 }	
					}
			}//For Task
	
		}
		boolean allResultsReady = false;
		while (!allResultsReady) {
			allResultsReady = true;
			
			for (int robotID : IDsIdleRobots) {
				for (int task : tasksIDs) {
					for(int path=0;path < maxNumPaths ; path ++) {
						if (!pathsToTargetGoal.containsKey(robotID*numTaskAug*maxNumPaths+task*maxNumPaths+path) )  {
							allResultsReady = false;
						}
					}
					
				}
				try { Thread.sleep(500); }
				catch (InterruptedException e) {
					e.printStackTrace();
				}
				}
				
		}
		metaCSPLogger.severe("All paths are computed.");
		long timeFinal = Calendar.getInstance().getTimeInMillis();
		long timeRequired = timeFinal- timeInitial;
		timeRequiretoEvaluatePaths = timeRequiretoEvaluatePaths + timeRequired;
		long timeInitial2 = Calendar.getInstance().getTimeInMillis();
		for (int robotID : robotsIDs) {
			int robotindex = robotsIDs.indexOf(robotID);
			double maxPathLength = 1;
			for (int taskID : tasksIDs ) {
				int taskIndex = tasksIDs.indexOf(taskID);
				double pathLength = MaxPathLength;
				//Evaluate path Length
				boolean typesAreEqual = false;
				 if (taskIndex < numTask && robotindex < numRobot ) {
				 typesAreEqual = taskQueue.get(taskIndex).isCompatible(coordinator.getRobot(robotID));
				 }
				 else {
					 //Considering a dummy robot or  a dummy task -> they don't have type
					 typesAreEqual = true;
				 }
				 for(int path = 0;path < maxNumPaths; path++) {
					 if(typesAreEqual) { 
						if(pathsToTargetGoal.get(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path)==null) {
							pathLength = MaxPathLength;
						}else {
							 pathLength=Missions.getPathLength(pathsToTargetGoal.get(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path));
						}
						
						 if ( pathLength > maxPathLength && pathLength != MaxPathLength) {
								maxPathLength = pathLength;
							}
					}
					 
					 PAll[robotindex][taskIndex][path] = pathLength;
				}//For path
			}//For task
			//Sum the max path length for each robot
			sumPathsLength += maxPathLength;
			//Sum the arrival time for the max path length
			sumArrivalTime += computeArrivalTime(maxPathLength,this.minMaxVel,this.minMaxAcc);
			}
		//Take the time to fill in the PAll Matrix
		long timeFinal2 = Calendar.getInstance().getTimeInMillis();
		long timeRequired2 = timeFinal2- timeInitial2;
		timeRequiretofillInPall = timeRequiretofillInPall + timeRequired2;
		double [][][] PAllAug =  checkTargetGoals(PAll);
		//Save the sum of max paths length to normalize path length cost
		this.sumMaxPathsLength = sumPathsLength;
		//Save the sum of arrival time considering max paths length to normalize delay cost
		this.sumArrivalTime = sumArrivalTime;
		//Return the cost of path length
		//Missions.saveScenario("ProvaScenario"+numAllocation);
		if(saveFutureAllocations==true) {
			numAllocation += 1;
		}
		
		
		for (int robotID : IDsIdleRobots) {
			int cont= 0;
			//fileStream.println("Missions>> " + Missions.getMissions(robotID).size());
			for (int taskID : tasksIDs ) {
				//fileStream.println("Missions online>> " + Missions.getMissions(robotID).size());
				if(Missions.getMissions(robotID) != null){
					if(cont < Missions.getMissions(robotID).size()) {
						Mission m1 = Missions.getMission(robotID, cont);
						Missions.removeMissions(m1);
				 		
				 		//cont +=1;
					}
				}
				
			}
			//fileStream.println("Missions again >> " + Missions.getMissions(robotID).size());
			
		}
		return PAll;
		}
	
	
	
	/**
* Evaluate the cost associated to time delay on completion of a task for a specific robot, due to interference with other robot
	 * and precedence constraints. The cost is evaluated considering the intersection between the path of robot i-th
	 * with the paths of other robots, considering the actual Assignment, but also paths related to already driving robot
	 * are considered.
	 * @param robot -> The i-th Robot
	 * @param task -> The j-th Task
	 * @param pathID -> The s-th path
	 * @param assignmentMatrix -> The Assignment Matrix related to a solution of the optimization problem
	 * @param tec -> an Abstract Trajectory Envelope Coordinator
	 * @return The cost associated to the delay on completion of task j for robot i due to interference with other robot
	 */
	public double evaluatePathDelay(int robotID ,int taskID,int pathID,double [][][] assignmentMatrix,AbstractTrajectoryEnvelopeCoordinator tec){
		CriticalSection[][][][] cssMatrix = new CriticalSection [IDsIdleRobots.size()][realTasksIDs.size()][maxNumPaths][1];

		long timeInitial2 = 0;

		int robotIndex = robotsIDs.indexOf(robotID);
		int taskIndex = tasksIDs.indexOf(taskID);
		//Evaluate the delay time on completion time for the actual couple of task and ID
		//Initialize the time delay 
		double delay = 0;
		//Considering the Actual Assignment 
		if (assignmentMatrix[robotIndex][taskIndex][pathID]>0) {
			
			// Only for real robots and tasks
			if (IDsIdleRobots.contains(robotID) && realTasksIDs.contains(taskID)) {
				//Take the Pose steering relate to i-th robot and j-th task from path set
				//PoseSteering[] pss1 = pathsToTargetGoalTotal.get((robot-1)*numTaskAug*maxNumPaths + task*maxNumPaths +pathID);
				PoseSteering[] pss1 = pathsToTargetGoal.get(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+pathID);
				if(pss1 == null) {
					return delay;
				}
				
				//Initialize Array of delays for the two robots
				TreeSet<IndexedDelay> te1TCDelays = new TreeSet<IndexedDelay>() ;
				TreeSet<IndexedDelay> te2TCDelays = new TreeSet<IndexedDelay>() ;
				//Compute the spatial Envelope for the i-th Robot
				
				SpatialEnvelope se1 = TrajectoryEnvelope.createSpatialEnvelope(pss1,tec.getFootprint(robotID));
				//Evaluate other path depending from the Assignment Matrix
				for(int secondRobotID : IDsIdleRobots) {
					int secondRobotIndex = IDsIdleRobots.indexOf(secondRobotID);
					for(int secondTaskID: realTasksIDs) {
						int secondTaskIndex = realTasksIDs.indexOf(secondTaskID);
						 for(int s = 0;s < maxNumPaths; s++) {
							
							 if (assignmentMatrix [secondRobotIndex][secondTaskIndex][s] > 0 && secondRobotID != robotID && secondTaskID != taskID) {
									//Take the path of this second robot from path set
									
								 	PoseSteering[] pss2 = pathsToTargetGoal.get(secondRobotID*numTaskAug*maxNumPaths  + secondTaskID*maxNumPaths+s);
									if (pss2 != null) {//is == null if robotType is different to Task type
										
										 
										//Evaluate the Spatial Envelope of this second Robot
										SpatialEnvelope se2 = TrajectoryEnvelope.createSpatialEnvelope(pss2,tec.getFootprint(secondRobotID));
										long timeInitial = Calendar.getInstance().getTimeInMillis();
										//Compute the Critical Section between this 2 robot
										CriticalSection [] css = new CriticalSection[1];
										
										/*
										
										if(criticalSections.containsKey(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+pathID) ) {
											css= criticalSections.get(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+pathID)[secondRobotIndex][secondTaskIndex][s];
											if(css.length == 1) {
												css = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, se2,true, Math.min(tec.getFootprintPolygon(robotID).getArea(),tec.getFootprintPolygon(secondRobotID).getArea()));
												
											}
										}else {
											css = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, se2,true, Math.min(tec.getFootprintPolygon(robotID).getArea(),tec.getFootprintPolygon(secondRobotID).getArea()));
											cssMatrix[secondRobotIndex][secondTaskIndex][s] = css;
											criticalSections.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+pathID, cssMatrix);
											//Evaluate the time to compute critical Section
											long timeFinal = Calendar.getInstance().getTimeInMillis();
											 long timeRequired = timeFinal- timeInitial;
											 timeRequiretoComputeCriticalSection = timeRequiretoComputeCriticalSection + timeRequired;
											 fileStream1.println(timeRequired+"");
					
										}
										
										*/
										
										
										css = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, se2,true, Math.min(tec.getFootprintPolygon(robotID).getArea(),tec.getFootprintPolygon(secondRobotID).getArea()));
										cssMatrix[secondRobotIndex][secondTaskIndex][s] = css;
										criticalSections.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+pathID, cssMatrix);
										//Evaluate the time to compute critical Section
										long timeFinal = Calendar.getInstance().getTimeInMillis();
										 long timeRequired = timeFinal- timeInitial;
										 timeRequiretoComputeCriticalSection = timeRequiretoComputeCriticalSection + timeRequired;
										 //fileStream1.println(timeRequired+"");
										
										 
										 timeInitial2 = Calendar.getInstance().getTimeInMillis();
										//Compute the delay due to precedence constraint in Critical Section
										for (int g = 0; g < css.length; g++) {
											Pair<Double, Double> a1 = estimateTimeToCompletionDelays(pss1.hashCode(),pss1,te1TCDelays,pss2.hashCode(),pss2,te2TCDelays, css[g]);
											double delayCriticalSection = Math.min(a1.getFirst(), a1.getSecond());
											//fileStream3.println(a1.getFirst() + " " +  a1.getSecond() + " " + robotID + " " + secondRobotID+ " ");
											if(delayCriticalSection < 0 ) {
												delay += 0;
											}else if(delayCriticalSection == Double.POSITIVE_INFINITY) {
												delay += 10000;
											}else {
												delay += delayCriticalSection;
											}
										}

								
								//Take the paths of driving robots from coordinator
							    pathsDrivingRobot = tec.getDrivingEnvelope();
							  //Evaluate the delay time due to already driving robots
							    for(int k = 0; k < pathsDrivingRobot.size(); k++) {
							    	CriticalSection [] cssDrivingRobot = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, pathsDrivingRobot.get(k),true, Math.min(tec.getFootprintPolygon(robotID).getArea(),tec.getFootprintPolygon(secondRobotID).getArea()));
							    	for (int b = 0; b < cssDrivingRobot.length; b++) {
										Pair<Double, Double> a1 = estimateTimeToCompletionDelays(pss1.hashCode(),pss1,te1TCDelays,pathsDrivingRobot.get(k).getPath().hashCode(),pathsDrivingRobot.get(k).getPath(),te2TCDelays, cssDrivingRobot[b]);
										double delayCriticalSection = Math.min(a1.getFirst(), a1.getSecond());
										if(delayCriticalSection < 0 ) {
											delay += 0;
										}else if(delayCriticalSection == Double.POSITIVE_INFINITY) {
											delay += 10000;
										}else {
											delay += delayCriticalSection;
										}
									}
							    }
							    
							    
							    long timeFinal2 = Calendar.getInstance().getTimeInMillis();
								long timeRequired2 = timeFinal2- timeInitial2;
								timeRequiretoComputePathsDelay = timeRequiretoComputePathsDelay + timeRequired2;
								//fileStream2.println(timeRequired2+"");
								}
							} 
						 }	
				}	
			}
				//fileStream3.println(" ");
				
			} else { //There also virtual robot and task are considered
				//the delay associated to dummy robot and task is considerd 0
				return delay;
			}
		}
		
		//return the delay for the i-th robot and the j-th task due to interference with other robots
		return delay;
		}
	
	
	
	/**
	 * Compute the arrival time to a task for a specified robot
	 * @param pathLength -> The max path for each robot
	 * @return The time to drive the path
	 */
	private double computeArrivalTime(double pathLength,double vel,double acc){
		//Compute the arrival time of this path, considering a robot alone with a velocity trapezoidal model
		double Pmax = Math.pow(vel, 2)/(2*acc);
		double arrivalTime = 1;
		if(pathLength > Pmax) {
			//Use trapezoidal profile
			 arrivalTime = pathLength/vel + vel/acc;
		}else {
			 arrivalTime = (2*pathLength)/vel;
		}
		
		//Return the arrival time 
		return arrivalTime;
	}
	
	/**
	 * Compute the arrival for all robots in fleet to all possible tasks
	 * @param pathLength -> The max path for each robot
	 * @return The time to drive the path
	 */
	private double [][][] computeArrivalTimeFleet(double[][][]PAll,AbstractTrajectoryEnvelopeCoordinator tec){
		//Compute the arrival time of this path, considering a robot alone with a velocity trapezoidal model
		double [][][] arrivalTimeMatrix = new double [numRobotAug][numTaskAug][maxNumPaths];
		for (int robotID : IDsIdleRobots ) {
			int i = robotsIDs.indexOf(robotID);
			for (int taskID : realTasksIDs ) {
				int j = tasksIDs.indexOf(taskID);
				 for(int path = 0;path < maxNumPaths; path++) {
					 //double vel = tec.getRobotMaxVelocity(robotID);
					// double acc = tec.getRobotMaxAcceleration(robotID);
					 double vel = tec.getForwardModel(robotID).getVel();
					 double acc = tec.getForwardModel(robotID).getAcc();
					 double arrivalTime = computeArrivalTime(PAll[i][j][path],vel,acc);
					 arrivalTimeMatrix[i][j][path] = arrivalTime;
				 }
				
			}
		}
		
		//Return the arrival time 
		return arrivalTimeMatrix;
	}
	
	
	/**
	 * Evaluate the tardiness in completion of a task for a single robot . The tardiness is the defined as the further time required to complete a task
	 * after the deadline 
	 * @param pathLength -> path length
	 * @param task -> the task j-th
	 * @return
	 */
	
	private double computeTardiness(int robotID,int taskID,double pathLength, AbstractTrajectoryEnvelopeCoordinator tec) {
		double tardiness = 0;
		if(realTasksIDs.contains(taskID)) {
			int taskIndex = realTasksIDs.indexOf(taskID);
			if (taskQueue.get(taskIndex).isDeadlineSpecified()) { // Compute tardiness only if specified in task constructor
				double deadline = taskQueue.get(taskIndex).getDeadline();  //Expressed in seconds
				//double vel = tec.getRobotMaxVelocity(robotID);
				//double acc = tec.getRobotMaxAcceleration(robotID);
				double vel = tec.getForwardModel(robotID).getVel();
				double acc = tec.getForwardModel(robotID).getAcc();
				double completionTime = computeArrivalTime(pathLength,vel,acc) + taskQueue.get(taskIndex).getOperationTime();
				tardiness = Math.max(0, (completionTime-deadline));
			}	
		}
		return tardiness;
	}
	
	
	/**
	 * Evaluate the tardiness in completion of a task . The tardiness is the defined as the further time required to complete a task
	 * after the deadline 
	 * @param pathLength -> path length
	 * @param task -> the task j-th
	 * @return
	 */
	
	private double[][][] computeTardinessFleet(double [][][]PAll,AbstractTrajectoryEnvelopeCoordinator tec) {
		double tardiness = 0;
		
		double [][][] tardinessMatrix = new double [numRobotAug][numTaskAug][maxNumPaths];
		for (int robotID : IDsIdleRobots ) {
			int i = robotsIDs.indexOf(robotID);
			for (int taskID : realTasksIDs ) {
				int j = tasksIDs.indexOf(taskID);
					for(int path = 0;path < maxNumPaths; path++) {
					if (taskQueue.get(j).isDeadlineSpecified()) { // Compute tardiness only if specified in task constructor
						double deadline = taskQueue.get(j).getDeadline();  //Expressed in seconds
						//double vel = tec.getRobotMaxVelocity(robotID);
						//double acc = tec.getRobotMaxAcceleration(robotID);
						double vel = tec.getForwardModel(robotID).getVel();
						double acc = tec.getForwardModel(robotID).getAcc();
						double completionTime = computeArrivalTime(PAll[i][j][path],vel,acc) + taskQueue.get(j).getOperationTime();
						tardiness = Math.max(0, (completionTime-deadline));
						tardinessMatrix[i][j][path] = tardiness;
						sumTardiness += tardiness;
					}	
				}
				
			}
		}
		return tardinessMatrix;
	}

	/**
	 * Evaluate the overall B function, that is the function that consider interference free costs
	 * Costs considered:
	 * 1) Path Length
	 * 2) Tardiness
	 * Each cost is already normalized;
	 * @param PAll
	 * @return
	 */
	private double [][][] evaluateBFunction(double [][][]PAll,AbstractTrajectoryEnvelopeCoordinator tec){
		double [][][] tardinessMatrix = computeTardinessFleet(PAll,tec);
		double [][][] BFunction = new double [numRobotAug][numTaskAug][maxNumPaths];
		costValuesMatrix = new double [numRobotAug][numTaskAug][maxNumPaths];
		if(linearWeight == 1) {
			double [][][] arrivalTimeMatrix = computeArrivalTimeFleet(PAll,tec);
			for (int i = 0 ; i < numRobotAug; i++) {
				for (int j = 0 ; j < numTaskAug; j++) {
					for(int path = 0;path < maxNumPaths; path++) {
						BFunction[i][j][path] = pathLengthWeight*PAll[i][j][path]/sumMaxPathsLength+ tardinessWeight*tardinessMatrix[i][j][path]/sumTardiness + arrivalTimeWeight*arrivalTimeMatrix[i][j][path]/sumArrivalTime;
						costValuesMatrix[i][j][path] = BFunction[i][j][path];
						//costValuesMatrix[i][j][path] =  PAll[i][j][path]/sumMaxPathsLength+ tardinessMatrix[i][j][path]/sumTardiness + arrivalTimeMatrix[i][j][path]/sumArrivalTime;
					}
					
				}
			}
		}
		else {
			for (int i = 0 ; i < numRobotAug; i++) {
				for (int j = 0 ; j < numTaskAug; j++) {
					for(int path = 0;path < maxNumPaths; path++) {
						BFunction[i][j][path] = pathLengthWeight*PAll[i][j][path]/sumMaxPathsLength+ tardinessWeight*tardinessMatrix[i][j][path]/sumTardiness;
						//costValuesMatrix[i][j][path] = PAll[i][j][path]/sumMaxPathsLength+ tardinessMatrix[i][j][path]/sumTardiness;
						costValuesMatrix[i][j][path] = BFunction[i][j][path] ;
					}
					

	
				}
			}
		}
		
		return BFunction;
	}
	
	/**
	 * Evaluates the number of all feasible solutions for an optimization problem, with is defined with {@link #buildOptimizationProblem}
	 * @param numRobot -> Number of Robot of the optimization problem 
	 * @param numTasks -> Number of Tasks of the optimization problem
	 * @return The number of all feasible solutions for the Optimization Problem
	 */
 
	public int numberFeasibleSolution(int numRobot,int numTasks){
		//Create an optimization problem
		MPSolver optimizationProblemCopy = buildOptimizationProblem(numRobot,numTasks);
		//Solve the optimization problem
	    MPSolver.ResultStatus resultStatus = optimizationProblemCopy.solve();
	    int numberFeasibleSolution = 0;
	    while(resultStatus != MPSolver.ResultStatus.INFEASIBLE ) {
			//Solve the optimization Problem
    		resultStatus = optimizationProblemCopy.solve();
    		//If The solution is feasible increment the number of feasible solution
    		if (resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
    			numberFeasibleSolution = numberFeasibleSolution+1;
    		}
    		double [][][] assignmentMatrix = saveAssignmentMatrix(numRobotAug,numTaskAug,optimizationProblemCopy);
			//Add the constraint to actual solution -> in order to consider this solution as already found  
    		optimizationProblemCopy = constraintOnPreviousSolution(optimizationProblemCopy,assignmentMatrix);
	    }
		//Return the Total number of feasible solution
	    optimizationProblemCopy.clear();
	    return numberFeasibleSolution;
	}
	

	/**
	 * Evaluates all possible feasible solutions for an optimization problem,with is defined with {@link #buildOptimizationProblem}. A feasible solution is a solution that verify constraints
	 * @param numRobot -> Number of Robots
	 * @param numTasks -> Number of Tasks
	 * @return A set containing all feasible solutions
	 */
	public double [][][][] evaluateFeasibleSolution(int numRobot,int numTasks){
		//Define the optimization problem
		MPSolver optimizationProblemCopy = buildOptimizationProblem(numRobot,numTasks);
		//Evaluate the number of all feasible solution for the optimization problem
	    int feasibleSolutions = numberFeasibleSolution(numRobot,numTasks);
	    //Initialize a set to store all feasible solution
		double [][][][] AssignmentMatrixOptimalSolutions = new double [feasibleSolutions][numRobot][numTasks][maxNumPaths]; 
	    ///////////////////////////////////////
	    for(int k=0; k < feasibleSolutions; k++) {
			//Solve the optimization problem
	    	MPSolver.ResultStatus resultStatus = optimizationProblemCopy.solve();
			//Transform the Assignment Vector to Matrix
			double [][][] AssignmentMatrix = saveAssignmentMatrix(numRobot,numTasks,optimizationProblemCopy);
			//Store the optimal solution
			AssignmentMatrixOptimalSolutions[k]=AssignmentMatrix;
			//Add the constraint to actual solution in order to consider this solution as already found  
			optimizationProblemCopy = constraintOnPreviousSolution(optimizationProblemCopy,AssignmentMatrix);
			}
		//Return the set of all Feasible solutions
	    return AssignmentMatrixOptimalSolutions;
		}

	
	
	/**
	 * Builds the optimization problem. Define a decision variable X_ij as a binary variable in which i indicate
	 * the robot id, j the tasks. Also constraints are defined:
	 * the constraints considered are :
	 * 1) Each Task can be assign only to a robot;
	 * 2) Each Robot can perform only a task at time;
	 * @param numRobot -> Number of Robots
	 * @param numTasksAug -> Number of Tasks.
	 * @return A constrained optimization problem without the objective function
	 */
	private MPSolver buildOptimizationProblem(int numRobotAug,int numTasksAug) {
		//Initialize a linear solver 
		
		
		MPSolver optimizationProblem = new MPSolver(
				"TaskAssignment", MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);
		
		//MPSolver optimizationProblem = new MPSolver(
				//"TaskAssignment", MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);
		//START DECISION VARIABLE VARIABLE
		MPVariable [][][] decisionVariable = new MPVariable[numRobotAug][numTasksAug][maxNumPaths];
		for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTasksAug; j++) {
				 for(int s = 0; s < maxNumPaths; s++) {
					 decisionVariable[i][j][s] = optimizationProblem.makeBoolVar("x"+"["+i+","+j+","+s+"]");
					
				 }
				
			 }
		}
		//END DECISION VARIABLE
		//////////////////////////
		// START CONSTRAINTS
		//Each Robot can be assign only to a Task	    
		 for (int i = 0; i < numRobotAug; i++) {
			 //Initialize the constraint
			 //MPConstraint c0 = optimizationProblem.makeConstraint(-Double.POSITIVE_INFINITY, 1);
			 MPConstraint c0 = optimizationProblem.makeConstraint(1, 1);
			 for (int j = 0; j < numTasksAug; j++) {
				 for(int s = 0; s < maxNumPaths; s++) {
					 //Build the constraint
					 c0.setCoefficient(decisionVariable[i][j][s], 1); 
				 }
				
			 }
		 }
		
		//Each task can be performed only by a robot
		 for (int j = 0; j < numTasksAug; j++) {
			//Initialize the constraint
			 MPConstraint c0 = optimizationProblem.makeConstraint(1, 1); 
			 for (int i = 0; i < numRobotAug; i++) {
				 for(int s = 0; s < maxNumPaths; s++) {
					 //Build the constraint
					 c0.setCoefficient(decisionVariable[i][j][s], 1); 
				 } 		
			 }
		 }
	
		 for (int robotID : robotsIDs ) {
				int i = robotsIDs.indexOf(robotID);
				for (int taskID : tasksIDs ) {
					int j = tasksIDs.indexOf(taskID);
					for(int s = 0; s < maxNumPaths; s++) {
							 if (i < numRobot) { //Considering only real Robot
								 PoseSteering[] pss = pathsToTargetGoal.get(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+s);
								 if(pss==null) {
									 MPConstraint c3 = optimizationProblem.makeConstraint(0,0);
									 c3.setCoefficient(decisionVariable[i][j][s],1); 
								 }
							 }
					}
				}
		 }
		 
		//END CONSTRAINTS
		//In case of having more task than robots, the task with a closest deadline are set with a higher priority
		 if(taskQueue.size() > IDsIdleRobots.size()) {
			 checkOnTaskDeadline();
			//Each task can be performed only by a robot
			 for (int j = 0; j < taskQueue.size(); j++) {
				//Initialize the constraint
				 if(taskQueue.get(j).isPriority()) {
					 MPConstraint c3 = optimizationProblem.makeConstraint(1, 1); 
					 for (int i = 0; i < IDsIdleRobots.size(); i++) {
						 for(int s = 0; s < maxNumPaths; s++) {
							 //Build the constraint
							 c3.setCoefficient(decisionVariable[i][j][s], 1); 
						 } 		
					 }
				 }
			 }
		 }
		/////////////////////////////////////////////////
		return optimizationProblem;	
	}
	
	
	
	/**
	 * Builds the optimization problem complete with Objective Function. Define a decision variable X_ij as a binary variable in which i indicate
	 * the robot id, j the tasks. Also constraints are defined:
	 * the constraints considered are :
	 * 1) Each Task can be assign only to a robot;
	 * 2) Each Robot can perform only a task at time.
	 * The objective function is defined as sum(c_ij * x_ij) for (i = 1...n)(j = 1...m)
	 * with n = number of robot and m = number of tasks.
	 * Only the B function is considered in this case, and each cost is normalized with the max path length considering
	 * all missions.
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 * @return A constrained optimization problem with the objective function and each cost is normalized
	 */
	public MPSolver createOptimizationProblem(AbstractTrajectoryEnvelopeCoordinator tec) {
		this.initialTime = 	Calendar.getInstance().getTimeInMillis();
	
		
		
		//Perform a check in order to avoid blocking
		//checkOnBlocking(tec);
		//Take the number of tasks
		numTask = taskQueue.size();
		//Get free robots and their IDs
		numRobot = tec.getIdleRobots().size();
		IDsIdleRobots = tec.getIdleRobots();
		//Evaluate dummy robot and dummy task
		checkOnBlocking();
		dummyRobotorTask(numRobot,numTask,tec);
		initializeRobotsIDs();
		initializeTasksIDs();
		
		
		double[][][] PAll = evaluatePAll();
		
		
		double[][][] BFunction = evaluateBFunction(PAll,tec);
		
		
		
		//Build the solver and an objective function
		MPSolver optimizationProblem = buildOptimizationProblem(numRobotAug,numTaskAug);
		
		
		
		MPVariable [][][] decisionVariable = tranformArray(optimizationProblem); 
	    /////////////////////////////////
	    //START OBJECTIVE FUNCTION		
	    MPObjective objective = optimizationProblem.objective();
    	 for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTaskAug; j++) {
				 for(int s = 0; s < maxNumPaths; s++) {
					 double pathLength  =  BFunction[i][j][s];
					 if ( pathLength != MaxPathLength) {
						 //Set the coefficient of the objective function with the normalized path length
						 objective.setCoefficient(decisionVariable[i][j][s], pathLength); 
					 }else { // if the path does not exists or the robot type is different from the task type 
						//the path to reach the task not exists
						//the decision variable is set to 0 -> this allocation is not valid
						MPConstraint c3 = optimizationProblem.makeConstraint(0,0);
						c3.setCoefficient(decisionVariable[i][j][s],1); 
					 }
				 }
			 }			 
		 }
		//Define the problem as a minimization problem
		objective.setMinimization();
		//END OBJECTIVE FUNCTION
		return optimizationProblem;	
	}
	
	

	
	/**
	 * Create and allocate the missions to the robots
	 * @param AssignmentMatrix -> An Assignment Matrix of the optimization problem
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 * @return An updated Trajectory Envelope Coordinator Simulation in which the mission for each
	 * robot is defined
	 */
	public boolean allocateTaskstoRobots(double [][][] AssignmentMatrix,AbstractTrajectoryEnvelopeCoordinator tec){
		System.out.println("Number of Robot : " + numRobot);
		System.out.println("Number of Task : " + numTask);
		System.out.println("Number of dummy Robot : " + dummyRobot);
		System.out.println("Number of dummy Task : " + dummyTask);
		System.out.println("Total Number of Robot : " + numRobotAug);
		System.out.println("Total Number of Task : " + numTaskAug);
		for (int robotID : robotsIDs ) {
			int i = robotsIDs.indexOf(robotID);
			for (int taskID : tasksIDs ) {
				int j = tasksIDs.indexOf(taskID);
				for(int s = 0; s < maxNumPaths; s++) {
					 if (AssignmentMatrix[i][j][s] > 0) {
						 if (i < numRobot) { //Considering only real Robot
							 PoseSteering[] pss = pathsToTargetGoal.get(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+s);
							 //PoseSteering[] pss = new PoseSteering[1];
							 //pss[0] = new PoseSteering(tec.getRobotReport(robotID).getPose(),0);
							 //For Dispatch mission
							 if (j < numTask && pss != null) {
								 removePath(pss.hashCode());
								 taskQueue.get(j).assignRobot(robotID);
								 taskQueue.get(j).setPaths(pss);
								 Mission[] robotMissions = taskQueue.get(j).getMissions();
								 viz.displayTask(taskQueue.get(j).getStartPose(), taskQueue.get(j).getGoalPose(),taskID, "red");
								 //tec.addMissions(new Mission(IDsIdleRobots[i],pss));
								 System.out.println("Task # "+ taskID + " is Assigned");
								 //tec.setTaskAssigned(robotID,taskID);
								 tec.addMissions(robotMissions);
							 }else {
								 System.out.println("Virtual Task # "+ taskID + " is Assigned to a real robot");
							 }
						 }else{
							
							 
							
							 System.out.println("Task # "+ taskID + " is not Assigned to a real robot");
							 
						 }
					 }
					//Remove path from the path set
					pathsToTargetGoal.remove(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+s);
				 }
				 
			 }
		 }
		
		//Remove Assigned Tasks from the set	
		int i = 0;
		int cont = 0;
		while (i < Math.min(numRobot, numTask)) {
			if (taskQueue.size() == 0 || taskQueue.size() <= i) {
				break;
			}
			if (taskQueue.get(i).isTaskAssigned()){
				taskQueue.remove(i);
				System.out.println("Task # "+ (cont+1) + " is removed ");
			}else {
				i = i+1;
			}
			cont +=1;	
			
		}
		 System.out.println("Remaining task: "+ taskQueue.size());
		 robotsIDs.removeAll(robotsIDs);
		 tasksIDs.removeAll(tasksIDs);
		 realTasksIDs.removeAll(realTasksIDs);
		 IDsIdleRobots.removeAll(IDsIdleRobots);
		 ScenarioAllocation = null;
		
		return true;
	}//End Task Assignment Function
	

	
	Callback cb = new Callback() {
		private long lastUpdate = Calendar.getInstance().getTimeInMillis();
		@Override
		public void performOperation() {
			long timeNow = Calendar.getInstance().getTimeInMillis();
			if (timeNow-lastUpdate > 1000) {
				lastUpdate = timeNow;
			}
		}
	};
	
	/**
	 * Start the Task Allocation Algorithm 
	 * @param rsp -> The motion planner that will be called for planning for any
	 * robot. 
	 * @param alpha -> the weight of B and F function in objective function. It is considered as
	 * B*alpha + (1-alpha)*F
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 */
	public void startTaskAssignment(AbstractTrajectoryEnvelopeCoordinator tec) {
		//Create meta solver and solver
		coordinator = tec;
		numRobot = coordinator.getIdleRobots().size();
		//Start a thread that checks and enforces dependencies at every clock tick
		this.setupInferenceCallback();

	}
	
	
	protected void setupInferenceCallback() {
		
		Thread TaskAssignmentThread = new Thread("Task Assignment") {
			private long threadLastUpdate = Calendar.getInstance().getTimeInMillis();
			@Override
			
			public void run() {
				while (true) {
					System.out.println("Thread Running");
					
					if (!taskQueue.isEmpty() && coordinator.getIdleRobots().size() > 2) {
						MPSolver solverOnline = createOptimizationProblem(coordinator);
						double [][][] assignmentMatrix = solveOptimizationProblem(solverOnline,coordinator, OptimizationSolverType.SYSTEMATIC_ALGORITHM);
						for (int i = 0; i < assignmentMatrix.length; i++) {
							int robotID = robotsIDs.get((i));
							for (int j = 0; j < assignmentMatrix[0].length; j++) {
								int taskID = tasksIDs.get((j));
								for(int s = 0; s < maxNumPaths; s++) {
									System.out.println("x"+"["+(i+1)+","+(j+1)+","+(s+1)+"]"+" is "+ assignmentMatrix[i][j][s]);
									if (assignmentMatrix[i][j][s] == 1) {
										System.out.println("Robot " + robotID +" is assigned to Task "+ taskID +" throw Path " + (s+1));
									}
								}
									
							} 
						}
						allocateTaskstoRobots(assignmentMatrix,coordinator);
						System.out.print("Task to be completed "+ taskQueue.size());
						solverOnline.clear();
						if(taskPosponedQueue.size() !=0) {
							taskQueue.addAll(taskPosponedQueue);
							taskPosponedQueue.removeAll(taskPosponedQueue);
						}
					}
					
					//Sleep a little...
					if (CONTROL_PERIOD_Task > 0) {
						try { 
							System.out.println("Thread Sleeping");
							Thread.sleep(CONTROL_PERIOD_Task); } //Thread.sleep(Math.max(0, CONTROL_PERIOD-Calendar.getInstance().getTimeInMillis()+threadLastUpdate)); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}

					long threadCurrentUpdate = Calendar.getInstance().getTimeInMillis();
					EFFECTIVE_CONTROL_PERIOD_task = (int)(threadCurrentUpdate-threadLastUpdate);
					threadLastUpdate = threadCurrentUpdate;
					
					if (cb != null) cb.performOperation();

				}
			}
		};
		TaskAssignmentThread.setPriority(Thread.MAX_PRIORITY);
		TaskAssignmentThread.start();
	}
	
	
	/** 
	 * Solve the optimization problem given as input considering both B and F Functions. The objective function is defined as sum(c_ij * x_ij) for (i = 1...n)(j = 1...m).
	 * with n = number of robot and m = number of tasks. The solver first finds the optimal solution considering only B function and then
	 * for this each solution (that is an assignment) evaluates the cost of F function. Then a new optimal solution considering only B is 
	 * computed and it is consider only if the cost of this new assignment considering only B is less than the min cost of previous assignments
	 * considering both F and B function
	 * @param optimizationProblem -> An optimization problem defined with {@link #buildOptimizationProblemWithB}
	 * @param tec -> an Abstract Trajectory Envelope Coordinator

	 * @return An Optimal Assignment that minimize the objective function
	 */
	
	public double [][][] solveOptimizationProblem(MPSolver optimizationProblem,AbstractTrajectoryEnvelopeCoordinator tec, OptimizationSolverType optimizationSolver ){
		
		
		double[][][] optimalAssignment = null;
		switch(optimizationSolver) {
		
		case SYSTEMATIC_ALGORITHM:
			SystematicAlgorithm systematicAlgorithm = new SystematicAlgorithm();
			optimalAssignment = systematicAlgorithm.solveOptimizationProblem(optimizationProblem,tec,this);
			this.ScenarioAllocation = null;
			this.scenario = null;
			break;
		case GRADIENT_DESCENT_ALGORITHM:
			break;
		case SIMULATED_ANNEALING_ALGORITHM:
			break;
		case GREEDY_ALGORITHM:
			break;
		case EXACT_ALGORITHM:
			break;
		default: 
			break;
		}
		return optimalAssignment;
	}
	
	
	
	}
