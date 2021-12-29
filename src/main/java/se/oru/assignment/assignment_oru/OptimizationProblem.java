package se.oru.assignment.assignment_oru;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Iterator;
import java.util.TreeSet;
import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;
import aima.core.util.datastructure.Pair;
import se.oru.assignment.assignment_oru.fleetmasterinterface.AbstractFleetMasterInterface;
import se.oru.assignment.assignment_oru.fleetmasterinterface.FleetMasterInterface;
import se.oru.assignment.assignment_oru.fleetmasterinterface.FleetMasterInterfaceLib.CumulatedIndexedDelaysList;
import se.oru.assignment.assignment_oru.methods.AbstractOptimizationAlgorithm;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.util.Missions;
import com.google.ortools.linearsolver.*;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;


/**
 * This class provides a method to generate an optimal assignment problem for a fleet of robots and a set of tasks . 
 	This class build the problem as an ST-ST-IA problem, i.e.:
	 * the constraints considered are :
	 * 1) Each Task can be assign only to a robot;
	 * 2) Each Robot can perform only a task at time;
 * @author pofe
 *
 */

public class OptimizationProblem extends AbstractOptimizationProblem{
	 


		//Parameters of weights in Optimization Problem for Interference free-cost function
		protected double pathLengthWeight = 1;
		protected double arrivalTimeWeight = 0;
		protected double tardinessWeight = 0;
		
		
		//Normalizing factors
		protected double sumTardiness = 1;
		protected double sumMaxPathsLength = 1; //This normalizing factor is obtained by summing the longest paths for each idle robot in the fleet
		protected double sumArrivalTime = 1; //This normalizing factor is the sum of arrival time of completing the longest path  for each robot
		
		protected double slowestRobotVelocity;
		protected double slowestRobotAcceleration;
		
		//Interference Parameters
		protected ArrayList <SpatialEnvelope> pathsDrivingRobots = new ArrayList <SpatialEnvelope>();

		
		protected int numAllocation = 1;
		
		
		
		//Parameters for time analysis
		protected long timeRequiretoEvaluatePaths;
		protected long timeRequiretofillInPall;
		protected long timeRequiretoComputeCriticalSection;
		protected long timeRequiretoComputePathsDelay;
		protected long initialTime;
			
		
		
		//FleetMaster Interface Parameters	
		protected AbstractFleetMasterInterface fleetMasterInterface = null;
		protected boolean propagateDelays = false;
		
		
		
		

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
			this.fleetMasterInterface.setDefaultFootprint(DEFAULT_FOOTPRINT);
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
			this.fleetMasterInterface.setDefaultFootprint(DEFAULT_FOOTPRINT);
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
						//indices.add(new Long(current.getIndex()));
						indices.add(Long.valueOf(current.getIndex()));
						values.add(current.getValue());
						
					}
					else if (prev.getIndex() == current.getIndex())				
						//Handle multiple delays in the same critical point
						values.set(values.size()-1, values.get(values.size()-1) + current.getValue());
					else {
						//Add the cumulative value if it is not the first.
						//indices.add(new Long(current.getIndex()));
						indices.add(Long.valueOf(current.getIndex()));
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
		 * Computes the minimum values of maximum velocity and acceleration considering all robots of the fleet. 
		 * This is used as a estimation of the longest nominal time (i.e. time without interference among robots) to complete a path. 
		 */
		
		protected void computeMaxVelandAccel() {
			double maxVel = Integer.MAX_VALUE;
			double maxAcc = Integer.MAX_VALUE;
			for(int robotID: coordinator.getIdleRobots()) {
				if(maxVel > coordinator.getRobotMaxVelocity(robotID)) {
					maxVel = coordinator.getRobotMaxVelocity(robotID);
				}
				if(maxAcc > coordinator.getRobotMaxAcceleration(robotID)){
					maxAcc = coordinator.getRobotMaxAcceleration(robotID);
				}
			}
			this.slowestRobotVelocity = maxVel;
			this.slowestRobotAcceleration = maxAcc;
		}
		
		
	
		/**
		 * Set the weights of cost functions considered in function B (i.e. all costs related to the single robot) for this Optimization Problem. These must be numbers between 0 and 1.
		 * Three costs functions are considered into B for this optimization problem: path length, nominal arrival time, tardiness. This function allow to set the weight (i.e. the importance) to each of then
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
	 * Evaluate the PAll matrix, that is a matrix that contains all path for each possible combination of robot
	 * and task
	 * If a path between a couple of robot and task does not exists, the cost is consider infinity.
	 * @return The PAll matrix
	 */
	protected double [][][] evaluatePAll(){
		
	
		if(this.slowestRobotAcceleration == 0 && this.slowestRobotVelocity == 0) {
			computeMaxVelandAccel();
		}
		
		//Evaluate the path length for the actual couple of task and ID
		//Initialize the sum of max paths lengths and time to do it for each robot
		//This cost are used then for normalizing cost
		double sumPathsLength = 0;
		double sumArrivalTime = 0;
		double [][][] PAll = new double[numRobotAug][numTaskAug][alternativePaths];
		
		
		
		long timeInitial = Calendar.getInstance().getTimeInMillis();
		if(scenario != null) {
			double [][][] PAllScenario = evaluatePAllWithScenario();
			return PAllScenario;
		}
		for (int robotID : robotsIDs) {
			
			//double maxPathLength = 1;
			//int robotIndex = robotsIDs.indexOf(robotID);
			for (int taskID : tasksIDs ) {
				int taskIndex = tasksIDs.indexOf(taskID);
				
				//Evaluate path Length
				boolean typesAreEqual = false;
				 if (IDsIdleRobots.contains(robotID) && realTasksIDs.contains(taskID) ) {
				 //typesAreEqual = taskQueue.get(taskIndex).isCompatible(coordinator.getRobot(robotID)); getRobotTypes
				 typesAreEqual = taskQueue.get(taskIndex).isCompatible(getRobotTypes(robotID));
				 }
				 else {
					 //Considering a dummy robot or  a dummy task -> they don't have type
					 typesAreEqual = true;
				 }
				 for(int path = 0;path < alternativePaths; path++) {
					 final int pathID = path;
					  if(typesAreEqual) { // only if robot and task have the same types
	
						 
						    // evaluatePathLength(robotID,taskID,pathID,tec);	
							new Thread("Robot" + robotID) {
								public void run() {
										evaluatePathLength(robotID,taskID,pathID);								
								}		
							}.start();
							//Take time to evaluate the path
				
					}			 
					 else {
							 pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+path, null);		
					 }	
					}
			}//For Task
	
		}
		boolean allResultsReady = false;
		while (!allResultsReady) {
			allResultsReady = true;
			
			for (int robotID : IDsIdleRobots) {
				for (int task : tasksIDs) {
					for(int path=0;path < alternativePaths ; path ++) {
						if (!pathsToTargetGoal.containsKey(robotID*numTaskAug*alternativePaths+task*alternativePaths+path) )  {
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
				double pathLength = Double.POSITIVE_INFINITY;
				//Evaluate path Length
				boolean typesAreEqual = false;
				 if (taskIndex < numberTasks && robotindex < numberRobots ) {
				 //typesAreEqual = taskQueue.get(taskIndex).isCompatible(coordinator.getRobot(robotID));
				 typesAreEqual = taskQueue.get(taskIndex).isCompatible(getRobotTypes(robotID));
				 }
				 else {
					 //Considering a dummy robot or  a dummy task -> they don't have type
					 typesAreEqual = true;
				 }
				 for(int path = 0;path < alternativePaths; path++) {
					 if(typesAreEqual) { 
						if(pathsToTargetGoal.get(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+path) != null) {
							pathLength = Missions.getPathLength(pathsToTargetGoal.get(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+path));
						}
						if ( pathLength > maxPathLength && pathLength != Double.POSITIVE_INFINITY) {
								maxPathLength = pathLength;
						}
					}
					 
					 PAll[robotindex][taskIndex][path] = pathLength;
				}//For path
			}//For task
			//Sum the max path length for each robot
			sumPathsLength += maxPathLength;
			//Sum the arrival time for the max path length
			sumArrivalTime += computeArrivalTime(maxPathLength,this.slowestRobotVelocity,this.slowestRobotAcceleration);
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
		//metaCSPLogger.info("Scenario is saved with name" + " scenario#" + numAllocation);
		//Missions.saveScenario("scenario# "+ numAllocation);
		if(saveFutureAllocations == true) {
			numAllocation += 1;
		}
		
		//Remove all the missions from Missions set stored in the coordinator.
		for (int robotID : IDsIdleRobots) {
			int cont = 0;
			//for (int taskID : tasksIDs ) {
				if(Missions.getMissions(robotID) != null){
					if(cont < Missions.getMissions(robotID).size()) {
						Mission m1 = Missions.getMission(robotID, cont);
						Missions.removeMissions(m1);
				 		
				 		//cont +=1;
					}
				}
				
			//}
			
		}
		return PAllAug;
		}
	
	
	
	
	
	protected synchronized double evaluatePathLength(int robotID , int taskID, int alternativePath){
		//Evaluate the path length for the actual couple of task and ID
		//Initialize the path length to infinity
		double pathLength = Double.POSITIVE_INFINITY;
		//take index positon of robotID in Robot set
		int robotindex = IDsIdleRobots.indexOf(robotID);
		// Only for real robots and tasks
		if (IDsIdleRobots.contains(robotID) && realTasksIDs.contains(taskID)) {
			//Take the state for the i-th Robot
			RobotReport rr = coordinator.getRobotReport(IDsIdleRobots.get(robotindex));
			if (rr == null) {
				metaCSPLogger.severe("RobotReport not found for Robot" + robotID + ".");
				throw new Error("RobotReport not found for Robot" + robotID + ".");
			}
			//Evaluate the path from the Robot Starting Pose to Task End Pose
			int taskIndex = realTasksIDs.indexOf(taskID);
			AbstractMotionPlanner rsp =  coordinator.getMotionPlanner(robotID).getCopy(true);
			
			rsp.setStart(rr.getPose());
			rsp.setGoals(taskQueue.get(taskIndex).getStartPose(),taskQueue.get(taskIndex).getGoalPose());
			rsp.setFootprint(coordinator.getFootprint(robotID));
			
			if (!rsp.plan()) {
				System.out.println("Robot" + robotID +" cannot reach the Target End of Task " + taskID);
				//the path to reach target end not exits
				pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+alternativePath, null);		
				//Infinity cost is returned 
				
				return pathLength;
				
			}			
			//If the path exists
			//Take the Pose Steering representing the path
			PoseSteering[] pss = rsp.getPath();
			
			System.out.println("Robot " +robotID +" taskID "+ taskID +" throw Path " + pss[pss.length-1].getX() + " " + pss[pss.length-1].getY() + " " + pss[pss.length-1].getTheta());
			
			
			//Add the path to the FleetMaster Interface -> this is necessary for F function
			addPath(robotID, pss.hashCode(), pss, null, coordinator.getFootprint(robotID)); 
			//SpatialEnvelope se1 = TrajectoryEnvelope.createSpatialEnvelope(pss,tec.getFootprint(robotID));
			//Geometry kk = se1.getPolygon();
			//addPath(robotID, pss.hashCode(), pss, kk, tec.getFootprint(robotID));
			
			//Save the path to Task in the path set
			pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+alternativePath, pss);
			//Take the Path Length
			Mission m1 = new Mission(robotID,pss);
			Missions.enqueueMission(m1);
			pathLength = Missions.getPathLength(pss);
			
		} else { //There also virtual robot and task are considered 
			//There are considered real robot and dummy task
			if (numberRobots >= numberTasks && IDsIdleRobots.contains(robotID)){ //dummy task -> The Robot receive the task to stay in starting position
				//The second condition is used in the special case in which we have that one robot cannot be 
				//assigned to any tasks due to its type, so we must add a dummy robot and a dummy task, but we 
				//Create the task to stay in robot starting position
				PoseSteering[] dummyTask = new PoseSteering[1];
				//Take the state for the i-th Robot
				RobotReport rr = coordinator.getRobotReport(IDsIdleRobots.get(robotindex));
				if (rr == null) {
					metaCSPLogger.severe("RobotReport not found for Robot" + robotID + ".");
					throw new Error("RobotReport not found for Robot" + robotID + ".");
				}
				//take the starting position of the robot
				dummyTask[0] = new PoseSteering(rr.getPose(),0);
				//Add the path to the FleetMaster Interface -> so it can be considered as an obstacle from 
				//the motion planner
				
				System.out.println("Robot " +robotID +" taskID "+ taskID +" throw Path " + dummyTask[dummyTask.length-1].getX() + " " + dummyTask[dummyTask.length-1].getY() + " " + dummyTask[dummyTask.length-1].getTheta());
				
				addPath(robotID, dummyTask.hashCode(), dummyTask, null, coordinator.getFootprint(robotID));
				//Save the path to Dummy Task 
				pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+alternativePath, dummyTask);		
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
				pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+alternativePath, dummyRobot);
				pathLength = 1;
				Mission m1 = new Mission(robotID,dummyRobot);
				Missions.enqueueMission(m1);
				return pathLength;
			}	
		}
		return pathLength;
	}
	

	
	
	
	/**
	 * Evaluate the 3D matrix Pall for all the possible combination of paths p_ij to reach a task j for a robot i, using a pre-computed Scenario
	 * @return the 3D matrix Pall of all possible paths, where each element of the matrix [i] [i] [p_ij] represents the length of the path for the robot i to reach the end of task j 
	 */
	protected double [][][] evaluatePAllWithScenario(){
		
		//Evaluate the path length for all the possible paths for the actual couple of task and robot
		//This cost are used then for normalizing the cost
		double sumPathsLength = 0;
		double sumArrivalTime = 0;

		Missions.loadScenario(scenario);
		
		double [][][] PAll = new double[numRobotAug][numTaskAug][alternativePaths];
		for (int robotID : robotsIDs) {
			int robotindex = robotsIDs.indexOf(robotID);
			double maxPathLength = 1;
			for (int taskID : tasksIDs ) {
				int taskIndex = tasksIDs.indexOf(taskID);
				double pathLength = Double.POSITIVE_INFINITY;
				//Evaluate path Length
				boolean typesAreEqual = false;
				 if (taskIndex < numberTasks && robotindex < numberRobots ) {
				 //typesAreEqual = taskQueue.get(taskIndex).isCompatible(coordinator.getRobot(robotID));
				 typesAreEqual = taskQueue.get(taskIndex).isCompatible(getRobotTypes(robotID));
				 
				 }
				 else {
					 //Considering a dummy robot or  a dummy task -> same types since there are fictitious 
					 typesAreEqual = true;
				 }
				 for(int path = 0;path < alternativePaths; path++) {
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
							 			pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+path, pss);
										pathLength = Missions.getPathLength(pss);
						
							 		 }
						 			missionNumber +=1;
							 		
							 		
							 	}
						 	}else {
						 		if (numberRobots >= numberTasks && IDsIdleRobots.contains(robotID)){
						 			PoseSteering[] pss = new PoseSteering[] {new PoseSteering(coordinator.getRobotReport(robotID).getPose(),0)};
						 			pathLength =1 ;
						 			pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+path, pss);
						 			//Missions.removeMissions(Missions.getMission(robotID, cont));
						 			missionNumber += 1;
						 			
						 			
						 		}else {
						 			PoseSteering[] pss = new PoseSteering[] {new PoseSteering(taskQueue.get(0).getGoalPose(),0)};
						 			pathLength =1 ;
						 			pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+path, pss);
						 		}
						 	}

							
							
						
						 if ( pathLength > maxPathLength && pathLength != Double.POSITIVE_INFINITY) {
								maxPathLength = pathLength;
							}
					}else {
						 pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+path, null);		
					}
	
					 PAll[robotindex][taskIndex][path] = pathLength;
				}//For path
			}//For task

			//Sum the max path length for each robot
			sumPathsLength += maxPathLength;
			//Sum the arrival time for the max path length
			sumArrivalTime += computeArrivalTime(maxPathLength,this.slowestRobotVelocity,this.slowestRobotAcceleration);
			}
		double [][][] PAllAug =  checkTargetGoals(PAll);
		//Save the sum of max paths length to normalize path length cost
		this.sumMaxPathsLength = sumPathsLength;
		//Save the sum of arrival time considering max paths length to normalize delay cost
		this.sumArrivalTime = sumArrivalTime;
		
		//Return the cost of path length	

		return PAllAug;
		}
	
	
	
	/**
	 * Evaluate the cost associated to time delay on completion of a task for a specific robot, due to interference with other robot
	 * and precedence constraints. The cost is evaluated considering the intersection between the path of robot i-th
	 * with the paths of other robots, considering the actual Assignment. Also paths related to already driving robot
	 * are considered.
	 * @param robot -> The i-th Robot
	 * @param task -> The j-th Task
	 * @param pathID -> The s-th path
	 * @param assignmentMatrix -> The Assignment Matrix related to a solution of the optimization problem
	 * @return The cost associated to the delay on completion of task j for robot i due to interference with other robot
	 */
	protected double evaluatePathDelay(int robotID ,int taskID,int pathID,double [][][] assignmentMatrix){
		CriticalSection[][][][] cssMatrix = new CriticalSection [IDsIdleRobots.size()][realTasksIDs.size()][alternativePaths][1];

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
				PoseSteering[] pss1 = pathsToTargetGoal.get(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+pathID);
				if(pss1 == null) {
					return delay;
				}
				
				//Initialize Array of delays for the two robots
				TreeSet<IndexedDelay> te1TCDelays = new TreeSet<IndexedDelay>() ;
				TreeSet<IndexedDelay> te2TCDelays = new TreeSet<IndexedDelay>() ;
				//Compute the spatial Envelope for the i-th Robot
				
				SpatialEnvelope se1 = TrajectoryEnvelope.createSpatialEnvelope(pss1,coordinator.getFootprint(robotID));
				//Evaluate other path depending from the Assignment Matrix
				for(int secondRobotID : IDsIdleRobots) {
					int secondRobotIndex = IDsIdleRobots.indexOf(secondRobotID);
					for(int secondTaskID: realTasksIDs) {
						int secondTaskIndex = realTasksIDs.indexOf(secondTaskID);
						 for(int s = 0;s < alternativePaths; s++) {
							
							 if (assignmentMatrix [secondRobotIndex][secondTaskIndex][s] > 0 && secondRobotID != robotID && secondTaskID != taskID) {
									//Take the path of this second robot from path set
									
								 	PoseSteering[] pss2 = pathsToTargetGoal.get(secondRobotID*numTaskAug*alternativePaths  + secondTaskID*alternativePaths+s);
									if (pss2 != null) {//is == null if robotType is different to Task type
										
										 
										//Evaluate the Spatial Envelope of this second Robot
										SpatialEnvelope se2 = TrajectoryEnvelope.createSpatialEnvelope(pss2,coordinator.getFootprint(secondRobotID));
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
										
										
										css = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, se2,true, Math.min(coordinator.getFootprintPolygon(robotID).getArea(),coordinator.getFootprintPolygon(secondRobotID).getArea()));
										cssMatrix[secondRobotIndex][secondTaskIndex][s] = css;
										//criticalSections.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+pathID, cssMatrix);
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
							    pathsDrivingRobots = coordinator.getDrivingEnvelope();
							  //Evaluate the delay time due to already driving robots
							    for(int k = 0; k < pathsDrivingRobots.size(); k++) {
							    	CriticalSection [] cssDrivingRobot = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, pathsDrivingRobots.get(k),true, Math.min(coordinator.getFootprintPolygon(robotID).getArea(),coordinator.getFootprintPolygon(secondRobotID).getArea()));
							    	for (int b = 0; b < cssDrivingRobot.length; b++) {
										Pair<Double, Double> a1 = estimateTimeToCompletionDelays(pss1.hashCode(),pss1,te1TCDelays,pathsDrivingRobots.get(k).getPath().hashCode(),pathsDrivingRobots.get(k).getPath(),te2TCDelays, cssDrivingRobot[b]);
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
	
	
	
	public double evaluateInterferenceCost(int robotID ,int taskID,int pathID,double [][][] assignmentMatrix) {
		
		double pathDelayCost = evaluatePathDelay(robotID,taskID,pathID,assignmentMatrix)/sumArrivalTime;
		return pathDelayCost;
	}
	
	
	
	protected double computeArrivalTime(double pathLength,double vel,double acc){
		//Compute the arrival time of this path, considering a robot alone with a velocity trapezoidal model
		
		if(this.slowestRobotAcceleration == 0 && this.slowestRobotVelocity == 0) {
			computeMaxVelandAccel();
		}
		
		if(pathLength == Double.POSITIVE_INFINITY) {
			pathLength = Integer.MAX_VALUE;
		}
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
	 * Compute the nominal (i.e. suppose that the robot is alone in the map) arrival time (i.e. time to reach the target position of a task). 
	 * @param PAll ->  paths length Matrix
	 * @return Cost nominal arrival time matrix 
	 */
	protected double [][][] computeArrivalTime(double[][][]PAll){
		//Compute the arrival time of this path, considering a robot alone with a velocity trapezoidal model
		double [][][] arrivalTimeMatrix = new double [numRobotAug][numTaskAug][alternativePaths];
		for (int robotID : IDsIdleRobots ) {
			int i = robotsIDs.indexOf(robotID);
			for (int taskID : realTasksIDs ) {
				int j = tasksIDs.indexOf(taskID);
				 for(int path = 0;path < alternativePaths; path++) {
					 double vel = coordinator.getRobotMaxVelocity(robotID);
					 double acc = coordinator.getRobotMaxAcceleration(robotID);			 
					 arrivalTimeMatrix[i][j][path] = computeArrivalTime(PAll[i][j][path],vel,acc);
				 }			
			}
		}
		
		//Return the arrival time 
		return arrivalTimeMatrix;
	}
	
	

/*	
	protected double computeTardiness(int robotID,int taskID,double pathLength) {
		double tardiness = 0;
		if(realTasksIDs.contains(taskID)) {
			int taskIndex = realTasksIDs.indexOf(taskID);
			if (taskQueue.get(taskIndex).isDeadlineSpecified()) { // Compute tardiness only if specified in task constructor
				double deadline = taskQueue.get(taskIndex).getDeadline();  //Expressed in seconds
				double vel = coordinator.getRobotMaxVelocity(robotID);
				double acc = coordinator.getRobotMaxAcceleration(robotID);
				double completionTime = computeArrivalTime(pathLength,vel,acc) + taskQueue.get(taskIndex).getOperationTime();
				tardiness = Math.max(0, (completionTime-deadline));
			}	
		}
		return tardiness;
	}
*/	
	
	/**
	 * Evaluate the tardiness in completion of a task, for all the task set . The tardiness is the defined as the further time required to complete a task
	 * after the deadline 
	 * @param PAll -> paths length Matrix
	 * @return Cost Tardiness matrix 
	 */
	
	protected double[][][] computeTardiness(double [][][]PAll) {
		double [][][] tardinessMatrix = new double [numRobotAug][numTaskAug][alternativePaths];
		for (int robotID : IDsIdleRobots ) {
			int i = robotsIDs.indexOf(robotID);
			for (int taskID : realTasksIDs ) {
				int j = tasksIDs.indexOf(taskID);
					for(int path = 0;path < alternativePaths; path++) {					
						//tardinessMatrix[i][j][path] = computeTardiness(robotID,taskID,PAll[i][j][path]);
						double tardiness = 0;
						if(realTasksIDs.contains(taskID)) {
							int taskIndex = realTasksIDs.indexOf(taskID);
							if (taskQueue.get(taskIndex).isDeadlineSpecified()) { // Compute tardiness only if specified in task constructor
								double deadline = taskQueue.get(taskIndex).getDeadline();  //Expressed in seconds
								double vel = coordinator.getRobotMaxVelocity(robotID);
								double acc = coordinator.getRobotMaxAcceleration(robotID);
								double completionTime = computeArrivalTime(PAll[i][j][path],vel,acc) + taskQueue.get(taskIndex).getOperationTime();
								tardiness = Math.max(0, (completionTime-deadline));
							}	
						}
						tardinessMatrix[i][j][path] = tardiness;
						sumTardiness += tardiness;			
				}		
			}
		}
		return tardinessMatrix;
	}
	/*
	protected double[][][] computeTardinessFleet(double [][][]PAll,AbstractTrajectoryEnvelopeCoordinator tec) {
		double tardiness = 0;
		
		double [][][] tardinessMatrix = new double [numRobotAug][numTaskAug][maxNumPaths];
		for (int robotID : IDsIdleRobots ) {
			int i = robotsIDs.indexOf(robotID);
			for (int taskID : realTasksIDs ) {
				int j = tasksIDs.indexOf(taskID);
					for(int path = 0;path < maxNumPaths; path++) {
					if (taskQueue.get(j).isDeadlineSpecified()) { // Compute tardiness only if specified in task constructor
						double deadline = taskQueue.get(j).getDeadline();  //Expressed in seconds
						double vel = tec.getRobotMaxVelocity(robotID);
						double acc = tec.getRobotMaxAcceleration(robotID);
						//double vel = tec.getForwardModel(robotID).getVel();
						//double acc = tec.getForwardModel(robotID).getAcc();
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
	*/
	/**
	 * Evaluate the overall B function, that is the function that consider interference free costs
	 * Costs considered:
	 * 1) Path Length
	 * 2) Tardiness
	 * Each cost is already normalized;
	 * @return The interference free cost matrix
	 */
	protected double [][][] evaluateBFunction(){
		double[][][] PAll = evaluatePAll();
		double [][][] tardinessMatrix = computeTardiness(PAll);
		double [][][] BFunction = new double [numRobotAug][numTaskAug][alternativePaths];
		interferenceFreeCostMatrix = new double [numRobotAug][numTaskAug][alternativePaths];
		if(linearWeight == 1) {
			double [][][] arrivalTimeMatrix = computeArrivalTime(PAll);
			for (int i = 0 ; i < numRobotAug; i++) {
				for (int j = 0 ; j < numTaskAug; j++) {
					for(int path = 0;path < alternativePaths; path++) {
						BFunction[i][j][path] = pathLengthWeight*PAll[i][j][path]/sumMaxPathsLength + tardinessWeight*tardinessMatrix[i][j][path]/sumTardiness + arrivalTimeWeight*arrivalTimeMatrix[i][j][path]/sumArrivalTime;
						interferenceFreeCostMatrix[i][j][path] = BFunction[i][j][path];
						//costValuesMatrix[i][j][path] =  PAll[i][j][path]/sumMaxPathsLength+ tardinessMatrix[i][j][path]/sumTardiness + arrivalTimeMatrix[i][j][path]/sumArrivalTime;
					}
					
				}
			}
		}
		else {
			for (int i = 0 ; i < numRobotAug; i++) {
				for (int j = 0 ; j < numTaskAug; j++) {
					for(int path = 0;path < alternativePaths; path++) {
						BFunction[i][j][path] = pathLengthWeight*PAll[i][j][path]/sumMaxPathsLength+ tardinessWeight*tardinessMatrix[i][j][path]/sumTardiness;
						//costValuesMatrix[i][j][path] = PAll[i][j][path]/sumMaxPathsLength+ tardinessMatrix[i][j][path]/sumTardiness;
						interferenceFreeCostMatrix[i][j][path] = BFunction[i][j][path] ;
					}
					

	
				}
			}
		}
		
		return BFunction;
	}
	
	
	/**
	 * Build the optimization problem. Define a decision variable X_ijp_{ijs} as a binary variable in which i indicate
	 * the robot id, j the tasks, and p_{ijs} is the s-th path for the robot i-th to reach the target position of task j. The problem is build as an ST-ST-IA problem, i.e.:
	 * the constraints considered are :
	 * 1) Each Task can be assign only to a robot;
	 * 2) Each Robot can perform only a task at time;
	 * @return A constrained optimization problem without the objective function
	 */
	protected MPSolver buildOptimizationProblem() {
		//Initialize a linear solver 
		
		
		MPSolver optimizationProblem = new MPSolver(
				"TaskAssignment", MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);
		
		//MPSolver optimizationProblem = new MPSolver(
				//"TaskAssignment", MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);
		//START DECISION VARIABLE VARIABLE
		MPVariable [][][] decisionVariable = new MPVariable[numRobotAug][numTaskAug][alternativePaths];
		for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTaskAug; j++) {
				 for(int s = 0; s < alternativePaths; s++) {
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
			 for (int j = 0; j < numTaskAug; j++) {
				 for(int s = 0; s < alternativePaths; s++) {
					 //Build the constraint
					 c0.setCoefficient(decisionVariable[i][j][s], 1); 
				 }
				
			 }
		 }
		
		//Each task can be performed only by a robot
		 for (int j = 0; j < numTaskAug; j++) {
			//Initialize the constraint
			 MPConstraint c0 = optimizationProblem.makeConstraint(1, 1); 
			 for (int i = 0; i < numRobotAug; i++) {
				 for(int s = 0; s < alternativePaths; s++) {
					 //Build the constraint
					 c0.setCoefficient(decisionVariable[i][j][s], 1); 
				 } 		
			 }
		 }
	
		 for (int robotID : robotsIDs ) {
				int i = robotsIDs.indexOf(robotID);
				for (int taskID : tasksIDs ) {
					int j = tasksIDs.indexOf(taskID);
					for(int s = 0; s < alternativePaths; s++) {
							 if (i < numberRobots) { //Considering only real Robot
								 PoseSteering[] pss = pathsToTargetGoal.get(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+s);
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
			 sortTaskByDeadline();
			//Each task can be performed only by a robot
			 for (int j = 0; j < taskQueue.size(); j++) {
				//Initialize the constraint
				 if(taskQueue.get(j).isPriority()) {
					 MPConstraint c3 = optimizationProblem.makeConstraint(1, 1); 
					 for (int i = 0; i < IDsIdleRobots.size(); i++) {
						 for(int s = 0; s < alternativePaths; s++) {
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
	 * Create the optimization problem with part of the Objective Function (Function B). Define a decision variable X_ijp_{ijs} as a binary variable in which i indicate
	 * the robot id, j the tasks, and p_{ijs} is the s-th path for the robot i-th to reach the target position of task j. The problem is build as an ST-ST-IA problem, i.e.:
	 * 1) Each Task can be assign only to a robot;
	 * 2) Each Robot can perform only a task at time.
	 * The objective function is defined as alpha*B + (1-alpha)*F 
	 * where B includes the cost related to single robot ({@link #evaluateBFunction}), while F ({@link #evaluateInterferenceCost}) includes all costs related to interference
	 * @return A constrained optimization problem
	 */
	protected MPSolver createOptimizationProblem() {
		this.initialTime = 	Calendar.getInstance().getTimeInMillis();
		//Perform a check in order to avoid blocking
		//checkOnBlocking(tec);
		//Take the number of tasks
		numberTasks = taskQueue.size();
		//Get free robots and their IDs
		numberRobots = coordinator.getIdleRobots().size();
		IDsIdleRobots = coordinator.getIdleRobots();
		//Evaluate dummy robot and dummy task
		checkOnBlocking();
		dummyRobotorTask();
		initializeRobotsIDs();
		initializeTasksIDs();
		double[][][] BFunction = evaluateBFunction();
		
		
		
		//Build the optimization problem
		MPSolver optimizationProblem = buildOptimizationProblem();
		
		
		//Modify the coefficients of the objective function
		MPVariable [][][] decisionVariable = tranformArray(optimizationProblem); 
	    /////////////////////////////////
	    //START OBJECTIVE FUNCTION		
	    MPObjective objective = optimizationProblem.objective();
    	 for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTaskAug; j++) {
				 for(int s = 0; s < alternativePaths; s++) {
					 double singleRobotCost  =  BFunction[i][j][s];
					 if ( singleRobotCost != Double.POSITIVE_INFINITY) {
						 //Set the coefficient of the objective function with the normalized path length
						 objective.setCoefficient(decisionVariable[i][j][s], singleRobotCost); 
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
	 * Allocate the tasks to the robots
	 * @param AssignmentMatrix -> An (sub)optimal assignment Matrix of the actual optimization problem
	 */
	protected void allocateTaskstoRobots(double [][][] AssignmentMatrix){
		getProblemInfo();
		for (int robotID : robotsIDs ) {
			int i = robotsIDs.indexOf(robotID);
			for (int taskID : tasksIDs ) {
				int j = tasksIDs.indexOf(taskID);
				for(int s = 0; s < alternativePaths; s++) {
					 if (AssignmentMatrix[i][j][s] > 0) {
						 if (i < numberRobots) { //Considering only real Robot
							 PoseSteering[] pss = pathsToTargetGoal.get(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+s);
							 //PoseSteering[] pss = new PoseSteering[1];
							 //pss[0] = new PoseSteering(tec.getRobotReport(robotID).getPose(),0);
							 //For Dispatch mission
							 if (j < numberTasks && pss != null) {
								 removePath(pss.hashCode());
								 taskQueue.get(j).assignRobot(robotID);
								 taskQueue.get(j).setPaths(pss);
								 Mission[] robotMissions = taskQueue.get(j).getMissions();
								 viz.displayTask(taskQueue.get(j).getStartPose(), taskQueue.get(j).getGoalPose(),taskID, "red");
								 //tec.addMissions(new Mission(IDsIdleRobots[i],pss));
								 metaCSPLogger.info("Task # "+ taskID + " is Assigned");
								 metaCSPLogger.info("Robot " + robotID +" is assigned to Task "+ taskID +" throw Path " + (s+1));
								 //tec.setTaskAssigned(robotID,taskID);
								 coordinator.addMissions(robotMissions);
							 }else {
								 metaCSPLogger.info("Virtual Task # "+ taskID + " is Assigned to a real robot");
							 }
						 }else{
							
							 
							
							 metaCSPLogger.info("Task # "+ taskID + " is not Assigned to a real robot");
							 
						 }
					 }
					//Remove path from the path set
					pathsToTargetGoal.remove(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+s);
				 }
				 
			 }
		 }
		
		//Remove Assigned Tasks from the set	
		int i = 0;
		int cont = 0;
		while (i < Math.min(numberRobots, numberTasks)) {
			if (taskQueue.size() == 0 || taskQueue.size() <= i) {
				break;
			}
			if (taskQueue.get(i).isTaskAssigned()){
				taskQueue.remove(i);
				metaCSPLogger.info("Task # "+ (cont+1) + " is removed from Task set");
			}else {
				i = i+1;
			}
			cont +=1;	
			
		}
		 metaCSPLogger.info("Remaining task: "+ taskQueue.size());
		 robotsIDs.removeAll(robotsIDs);
		 tasksIDs.removeAll(tasksIDs);
		 realTasksIDs.removeAll(realTasksIDs);
		 IDsIdleRobots.removeAll(IDsIdleRobots);
		 ScenarioAllocation = null;
		
	}//End Task Assignment Function
	
	
	
	public void startTaskAssignment(AbstractOptimizationAlgorithm optimizationSolver ) {
		//Create meta solver and solver
		this.optimizationSolver = optimizationSolver;
		numberRobots = coordinator.getIdleRobots().size();
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
						optimizationModel = createOptimizationProblem();
						double [][][] assignmentMatrix = findOptimalAssignment(optimizationSolver);
						for (int i = 0; i < assignmentMatrix.length; i++) {
							int robotID = robotsIDs.get((i));
							for (int j = 0; j < assignmentMatrix[0].length; j++) {
								int taskID = tasksIDs.get((j));
								for(int s = 0; s < alternativePaths; s++) {
									System.out.println("x"+"["+(i+1)+","+(j+1)+","+(s+1)+"]"+" is "+ assignmentMatrix[i][j][s]);
									if (assignmentMatrix[i][j][s] == 1) {
										System.out.println("Robot " + robotID +" is assigned to Task "+ taskID +" throw Path " + (s+1));
									}
								}
									
							} 
						}
						allocateTaskstoRobots(assignmentMatrix);
						System.out.print("Task to be completed "+ taskQueue.size());
						optimizationModel.clear();
						if(taskPosponedQueue.size() !=0) {
							taskQueue.addAll(taskPosponedQueue);
							taskPosponedQueue.removeAll(taskPosponedQueue);
						}
					}
					
					//Sleep a little...
					if (CONTROL_PERIOD_TASK > 0) {
						try { 
							System.out.println("Thread Sleeping");
							Thread.sleep(CONTROL_PERIOD_TASK); } //Thread.sleep(Math.max(0, CONTROL_PERIOD-Calendar.getInstance().getTimeInMillis()+threadLastUpdate)); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}

					long threadCurrentUpdate = Calendar.getInstance().getTimeInMillis();
					EFFECTIVE_CONTROL_PERIOD_TASK = (int)(threadCurrentUpdate-threadLastUpdate);
					threadLastUpdate = threadCurrentUpdate;
					
					if (cb != null) cb.performOperation();

				}
			}
		};
		TaskAssignmentThread.setPriority(Thread.MAX_PRIORITY);
		TaskAssignmentThread.start();
		
	}
	
	
	
	
	} //End class

