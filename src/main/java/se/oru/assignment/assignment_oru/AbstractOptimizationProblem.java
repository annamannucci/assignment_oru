package se.oru.assignment.assignment_oru;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;

import java.util.List;

import java.util.logging.Logger;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.utility.UI.Callback;
import org.metacsp.utility.logging.MetaCSPLogging;
import com.vividsolutions.jts.geom.Coordinate;

import com.vividsolutions.jts.geom.Polygon;

import se.oru.assignment.assignment_oru.methods.AbstractOptimizationAlgorithm;
import se.oru.assignment.assignment_oru.util.TaskFleetVisualization;
import se.oru.assignment.assignment_oru.util.robotType.ROBOT_TYPE;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeCoordinator;

import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotReport;

import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.StringUtils;

import com.google.ortools.linearsolver.*;



/**
 * This class provides a framework to generate an optimal assignment problem for a fleet of robots and a set of tasks . 
 * An instantiatable {@link AbstractOptimizationProblem} must provide an implementation of the {@link #buildOptimizationProblem()} function to create the optimization problem, and an implementations of
 * the {@link #evaluateBFunction()} and {@link #evaluateInterferenceCost} methods to evaluate the interference free cost and the interference cost respectively.
 * @author pofe
 *
 */


public abstract class AbstractOptimizationProblem {
	 
	

		public static String TITLE = "assignment_oru - Robot-agnostic online task assignment for multiple robots";
		public static String COPYRIGHT = "Copyright \u00a9 2020-" + Calendar.getInstance().get(Calendar.YEAR) + " Paolo Forte";
		public static String[] CONTRIBUTORS = {"Anna Mannucci", "Federico Pecora"};
	
		//null -> public (GPL3) license
		public static String LICENSE = null;
	
		public static String PUBLIC_LICENSE = "This program comes with ABSOLUTELY NO WARRANTY. "
				+ "This program is free software: you can redistribute it and/or modify it under the "
				+ "terms of the GNU General Public License as published by the Free Software Foundation, "
				+ "either version 3 of the License, or (at your option) any later version. see LICENSE for details.";
		public static String PRIVATE_LICENSE = "This program comes with ABSOLUTELY NO WARRANTY. "
				+ "This program has been licensed to " + LICENSE + ". The licensee may "
				+ "redistribute it under certain conditions; see LICENSE for details.";
	
		//Force printing of (c) and license upon class loading
		static { printLicense(); }
	
		
	
		//Optimization Problem Parameters
		protected int numberRobots;
		protected int numberTasks;
		protected int dummyRobot;
		protected int dummyTask;
		protected int numRobotAug;
		protected int numTaskAug;
		protected int alternativePaths = 1;
		protected double linearWeight = 1;
		protected double [][][] interferenceFreeCostMatrix;
		protected ArrayList <Task> taskQueue = new ArrayList <Task>();
		protected ArrayList <Task> taskPosponedQueue = new ArrayList <Task>();	
		protected ArrayList <Integer> IDsIdleRobots = new ArrayList <Integer>();
		protected ArrayList <Integer> realTasksIDs = new ArrayList <Integer>();
		protected ArrayList <Integer> robotsIDs = new ArrayList <Integer>(); //this is the set of IDs of all the robots considered into the problem (i.e. both real and virtual robots)
		protected ArrayList <Integer> tasksIDs = new ArrayList <Integer>(); //this is the set of IDs of all the tasks considered into the problem (i.e. both real and virtual tasks)
		protected HashMap<Integer,ROBOT_TYPE> robotTypes = new HashMap<Integer,ROBOT_TYPE>();
		
		protected int virtualRobotID = Integer.MAX_VALUE; 
		protected int virtualTaskID = Integer.MAX_VALUE;
		
		
		protected double [][][] optimalAssignment;
		protected MPSolver optimizationModel;
		protected List <double [][][]> feasibleSolutions = new ArrayList <double [][][]>();
		
		//ROADMAP Parameters
		protected String scenario;
		protected double [][][] ScenarioAllocation;
		protected boolean saveFutureAllocations = false;
		
		
		
		//Motion planner and Coordinator Parameters
		protected AbstractTrajectoryEnvelopeCoordinator coordinator;
		protected Logger metaCSPLogger = MetaCSPLogging.getLogger(this.getClass());	
		protected HashMap<Integer, PoseSteering[]> pathsToTargetGoal =  new HashMap<Integer, PoseSteering[]>();
		
	
		
	
		
		//Thread Parameters 
		protected int CONTROL_PERIOD_TASK = 20000;
		public static int EFFECTIVE_CONTROL_PERIOD_TASK = 0;	
		protected AbstractOptimizationAlgorithm optimizationSolver;
		
		
		//Visualization parameters
		protected TaskFleetVisualization viz = null;
		
		
		
		/**
		 * The default footprint used for robots if none is specified.
		 * NOTE: coordinates in footprints must be given in in CCW or CW order. 
		 */
		public static Coordinate[] DEFAULT_FOOTPRINT = new Coordinate[] {
				new Coordinate(-1.7, 0.7),	//back left
				new Coordinate(-1.7, -0.7),	//back right
				new Coordinate(2.7, -0.7),	//front right
				new Coordinate(2.7, 0.7)	//front left
		};
		
		
		
		

		 /**
		 * Return the biggest footprint of the robots in fleet
		*/
			
		protected Coordinate[] getMaxFootprint() {
			double maxArea = 0.0;
			int robotIDMax = 1;
			for(int it :  IDsIdleRobots){
				double robotFootprintArea = coordinator.getFootprintPolygon(it).getArea();
				if(robotFootprintArea > maxArea) {
					maxArea = robotFootprintArea;
					robotIDMax = it; 
				}
			}
			return coordinator.getFootprintPolygon(robotIDMax).getCoordinates();
		}
	
		
		/**
		 * Set the type for the specific robot 
		 * @param robotID -> ID of the robot
		 * @param robotType -> type of the robot expressed as {@link ROBOT_TYPE} 
		 */
		public void setRobotType(int robotID, ROBOT_TYPE robotType){
			if(!this.robotTypes.containsKey(robotID)){
				this.robotTypes.put(robotID, robotType);
			}
			else {
				this.robotTypes.replace(robotID, robotType);
			}
			
		}
		
		
		/*
		public void setRobotTypes(int robotID, int robotType){
			ArrayList<Integer> rbTypes = new ArrayList<Integer>(); 
			rbTypes.add(robotType);
			setRobotTypes(robotID,rbTypes);
			
		}
			
			
			
		public void setRobotTypes(int robotID, ArrayList<Integer> robotTypes){
			if(!this.robotTypes.containsKey(robotID)){
				this.robotTypes.put(robotID, robotTypes);
			}
			else {
				ArrayList<Integer> rbTypes = this.robotTypes.get(robotID);
				rbTypes.addAll(robotTypes);
				this.robotTypes.replace(robotID, rbTypes);
			}
			
		}
		*/
		/**
		 * Get the robot type of the specific robot.
		 * @param robotID -> the ID of the robot
		 * @return The robot type of the specific robot.
		 */
		
		public ROBOT_TYPE getRobotTypes(int robotID){
			if(this.robotTypes.containsKey(robotID)) {
				return this.robotTypes.get(robotID);
			}
			return null;
		}
			
		
		/**
		 * Save scenarios (Missions + paths) for all the optimization problems that will be solved (if some tasks are assigned online)
		 * 
		 */
		public void saveAllScenarios() {
			this.saveFutureAllocations = true;
			
		}
		
				
		/**
		 * Set the maximum alternative number of paths to reach a goal for each robot.
		 * @param alternativePaths -> number of path to reach a goal
		 */
		public void setmaxNumberOfAlternativePaths(int alternativePaths) {
			
			this.alternativePaths = alternativePaths;
		}
		
		
		/** Get the maximum alternative number of paths to reach a goal for each robot.
		 */
		public int getmaxNumberOfAlternativePaths() {
			
			return this.alternativePaths;
		}
		
		/**
		 * Get the interference free cost matrix associated to the problem (B function)
		 * @return
		 */
		public double [][][] getInterferenceFreeCostMatrix() {
			return this.interferenceFreeCostMatrix;
		}
		
		
		/**
		 * Save the assignment Matrix in the specific file. If the file does not exist, it will be created.
		 * @param filename -> name of the file where to save the matrix
		 * @param optimalAssignmentMatrix -> the task assignment matrix
		 */
		public void saveAssignmentMatrixinFile(String filename, double[][][] optimalAssignmentMatrix) {
		    try {
		        BufferedWriter bw = new BufferedWriter(new FileWriter((filename),true));
		        bw.write("{{");
		        for (int i = 0; i < optimalAssignmentMatrix.length; i++) {
		        	for (int j = 0; j < optimalAssignmentMatrix[i].length; j++) {
		        		bw.write("{");
		        		for (int s = 0; s < alternativePaths; s++) {
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
		 * Load a Scenario (paths + missions) 
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
		 * Set the Coordinator. The coordinator will be used to manage the fleet and solve the coordination problem (avoid collision)
		 * @param coordinator -> An instance of a AbstractTrajectoryEnvelopeCoordinator
		 */
		
		public void setCoordinator(AbstractTrajectoryEnvelopeCoordinator coordinator) {
			this.coordinator = coordinator;
		}
		
		
		/**
		 * Get the optimal Assignment Matrix computed with {@link #solveProblem} associated to the problem
		 * @return optimalAssignment -> Optimal Task assignment
		 */
		
		public double [][][] getOptimalAssignment(){
			return optimalAssignment;
		}
		
		/**
		 * Set the linear weight used in Optimization Problem. This parameter sets the weight (i.e. the importance) of B and F. If alpha = 1 (default value) only the B function is minimized.
		 * linearWeight must be > 0 and < 1. 
		 * More this value is close to 0, more is the importance given to F Function. More this value is close to 1, more is the importance given to B function.
		 * @param alpha -> the linear weight value.
		 */
		
		public void setLinearWeight(double alpha) {
			if(alpha < 0 || alpha > 1) {
				throw new Error("The linear weigth must be > 0 and < 1!");
			}
			metaCSPLogger.info("alpha is set to : " + alpha);
			this.linearWeight = alpha;
		}	
		
		
		/**
		 * Get the linear weight used in Optimization Problem.This parameter sets the weight of B and F. If alpha = 1 (default value) only the B function is minimized.
		 * @return the value of the linear weight alpha
		 */
		
		public double getLinearWeight() {
			return this.linearWeight;
		}	
		
		
	/**
	 * Set the Fleet Visualization.
	 * @param viz -> An instance of a TaskFleetVisualization
	 */
			
	public void setFleetVisualization(TaskFleetVisualization viz) {
		this.viz = viz;
	}
	
	/**
	 * Initialize the IDs of all the robots considered into the problem (both real and virtual)
	 */
	
	protected void initializeRobotsIDs() {
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
	 * Check if a goal can be reached by at least one robot of the Fleet . If not, a dummy robot and task are added for each 
	 * unreachable location, to make the problem square (equal number of robot and task). If one location cannot be reached
	 * the task associated to that location will be assigned to a virtual robot, while a virtual task will be associated to 
	 * a real robot (which one will depend on other tasks).
	 * @param PAll -> the 3D matrix of all paths 
	 * @return An updated PAll incremented by 1 on each direction (i.e. i, j, p_ij)
	 */
	
	protected  double [][][] checkTargetGoals (double [][][] PAll){
		for (int j= 0; j< PAll[0].length ; j++) {
			boolean targetEndCanBeReach = false;
			for (int i = 0; i < PAll.length; i++) {
				for (int s = 0; s < alternativePaths; s++) {
					if(PAll[i][j][s] != Double.POSITIVE_INFINITY) {
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
		
		double [][][] PAllAug = new double [numRobotAug][numTaskAug][alternativePaths];
		int robotID = 0;
		int taskID = 0;
		for(int i = 0;i < numRobotAug; i++) {
			for(int j = 0; j<  numTaskAug;j++) {
				for (int s = 0; s < alternativePaths; s++) {
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
						pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+s, null); //FIXME null may be not correct
				
					}
				}
					
			}
		}
		return PAllAug;
	}
	
	
	/**
	 * Sort the set of tasks task by deadlines
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
	
	protected void sortTaskByDeadline() {
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
	 * Delete one or multiple tasks from the queue to avoid blocking. Blocking happens a task execution is blocked by the execution 
	 * of another task (i.e. two tasks may have a common or near goal location). 
	 * The deleted task will be postponed and execute later. If deadlines are present, them will be considering into the evaluation
	 * (the task with the shortest deadline will be prioritized)
	*/
	
	protected void checkOnBlocking() { 
		int tasksPosponed = 0;
		Coordinate[] taskFootprint = getMaxFootprint();
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
			Polygon ll = TrajectoryEnvelope.createFootprintPolygon(Taskfootprint1,Taskfootprint2,Taskfootprint3,Taskfootprint4);
			
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
							tasksPosponed += 1;
						}else {
							if (taskProva2.getDeadline() == -1 && currentTask.getDeadline() != -1){
								taskQueue.remove(k);
								taskPosponedQueue.add(taskProva2);
								tasksPosponed += 1;

							}else if(taskProva2.getDeadline() != -1 && currentTask.getDeadline() == -1) {
								taskQueue.remove(j);
								taskPosponedQueue.add(currentTask);
								tasksPosponed += 1;
								j -=1;
								break;
							}else {
								if(taskProva2.getDeadline() > currentTask.getDeadline() ) {
									taskQueue.remove(k);
									taskPosponedQueue.add(taskProva2);
									tasksPosponed += 1;
								}else {
									taskQueue.remove(j);
									taskPosponedQueue.add(currentTask);
									tasksPosponed += 1;
									j -=1;
									break;
								}	
							}
						}
						
					}
					
				}
			}
		} //end for loop
		metaCSPLogger.info("Task posponed for avoid blocking : " + tasksPosponed);
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
		metaCSPLogger.info(task.toString() +  "  is added" );
		return taskQueue.add(task);
	}

    

	/**
	 * Compute the number of Dummy Robots and/or Tasks to add to the problem. Consider the possibility to have a different number of robots (N) and tasks (M). If N > M, dummy tasks are 
	 * considered, where a dummy task is a task for which a robot stay in starting position; while if M > N dummy robots are considered, where a dummy robot is only a virtual robot. 
	 */
	protected void dummyRobotorTask() {
		numRobotAug = numberRobots;
		numTaskAug = numberTasks;
		//Restore initial value for dummy robot and task
		dummyTask = 0;
		dummyRobot = 0;
		//Considering the possibility to have n != m
		//First check is related to numbers of real robots and tasks associated to the problem
		//If n > m -> we have dummy robot
		if (numberRobots > numberTasks) {
			dummyTask = numberRobots - numberTasks;
			numTaskAug = numberTasks + dummyTask;
		}
		//If n < m -> we have dummy tasks
		else if (numberRobots < numberTasks) {
			dummyRobot = numberTasks - numberRobots;
			numRobotAug = numberRobots + dummyRobot;
		}
		//A second check is : check if a robot cannot perform at least one task of the set
		//A dummy robot and task will be added for each robot in this case 
		if (dummyTask == 0 || dummyRobot != 0) {
			for (int i = 0; i < numberRobots; i++) {
				boolean canExecuteATask = false;
				 for (int j = 0; j < numberTasks; j++) {
					 //check if robot can be assigned to one task
					 //if (taskQueue.get(j).getTaskType() == tec.getRobotType(IDsIdleRobots[i])) {getRobotTypes
					 //if (taskQueue.get(j).isCompatible(tec.getRobot(IDsIdleRobots.get(i)))) {
					 if (taskQueue.get(j).isCompatible(getRobotTypes(IDsIdleRobots.get(i)))) {
						 canExecuteATask = true;
						 
					 }
				 }
				 //the robot cannot be assigned to any task -> add a dummy robot and task
				 if (!canExecuteATask) {
					 dummyRobot += 1 ;
					 dummyTask += 1 ;
					 numRobotAug += 1;
					 numTaskAug += 1;
				 }
			}
		}
		//If A task cannot be assigned to any robot
		if (dummyRobot == 0 || dummyTask != 0) {
			for (int i = 0; i < numberTasks; i++) {
				boolean flagAllocateTask = false;
				 for (int j = 0; j < numberRobots; j++) {
					//check if task can be assigned to one robot
					 //if (taskQueue.get(i).getTaskType() == tec.getRobotType(IDsIdleRobots[j])) {
					 //if (taskQueue.get(i).isCompatible(tec.getRobot(IDsIdleRobots.get(j)))) {
					if (taskQueue.get(i).isCompatible(getRobotTypes(IDsIdleRobots.get(j)))) {	 
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
	 * @param optimizationProblem -> The optimization problem defined with {@link #createOptimizationProblem }
	 * @return 3D Matrix of Decision Variable of the input problem
	*/
	protected MPVariable [][][] tranformArray(MPSolver optimizationProblem) {
		//Take the vector of Decision Variable from the Optimization Problem
		MPVariable [] array1D = optimizationProblem.variables();
		MPVariable [][][] decisionVariable = new MPVariable [numRobotAug][numTaskAug][alternativePaths];
		//Store them in a 2D Matrix
	    for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTaskAug; j++) {
				 for (int s = 0; s < alternativePaths; s++) {
					 decisionVariable[i][j][s] = array1D[i*numTaskAug*alternativePaths+j*alternativePaths+s];
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
    			for(int s = 0;s < alternativePaths; s++) {
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
	 * Impose a constraint on the optimization problem on previous optimal solution cost in order to prune all solutions with a cost higher
	 * than objectiveValue . In this manner is possible to avoid some cases.
	 * @param optimizationProblem -> An optimization problem  defined with {@link #buildOptimizationProblem}
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
    			for(int s = 0;s < alternativePaths; s++) {

    				c3.setCoefficient(decisionVariable[i][j][s],interferenceFreeCostMatrix[i][j][s]);
    			}
    		}		
		 }
    	//Return the updated Optimization Problem
    	return optimizationProblem;
	}
	
	
	/**
	 * Get the assignment Matrix of a optimization problem. This method need to be called after {@link #findOptimalAssignment} method.
	 * @param numRobot -> Number of robots
	 * @param numTasks -> Number of tasks
	 * @param optimizationProblem -> A solved optimization problem
	 * @return Assignment matrix for the optimization problem given as input
	 */
	
	public double [][][] getAssignmentMatrix(MPSolver optimizationProblem){
		//Take the decision variable from the optimization problem
		MPVariable [][][] decisionVariable = tranformArray(optimizationProblem);
		double [][][] assignmentMatrix = new double [numRobotAug][numTaskAug][alternativePaths];	
		//Store decision variable values in a Matrix
		for (int i = 0; i < numRobotAug; i++) {
			for (int j = 0; j < numTaskAug; j++) {
				for(int s = 0;s < alternativePaths; s++) {
					assignmentMatrix[i][j][s] = decisionVariable[i][j][s].solutionValue();
				}
			}
		}
		return assignmentMatrix;	
	}
	
	
	/**
	 * Evaluate the path length for the a couple of robot and task.
	 * If a path between a couple of robot and task does not exists the cost is consider infinity.
	 * @param robotID -> The  Robot ID
	 * @param taskID -> The Task ID
	 * @param alternativePath -> The path ID
	 * @return The length of the specific path for the robotID to reach the target position of task taskID
	 */
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
			//addPath(robotID, pss.hashCode(), pss, null, coordinator.getFootprint(robotID)); 
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
				
				//addPath(robotID, dummyTask.hashCode(), dummyTask, null, coordinator.getFootprint(robotID));
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
	 * Evaluate the interference cost for the specific robot (robotID) to perform the  specific task (taskID) following the specific path (pathID)
	 * @param robotID -> ID of the robot that perform the task
	 * @param taskID -> ID of the task that the robot will perform
	 * @param pathID -> ID of the path that the robot will follow
	 * @param assignmentMatrix -> Assignment Matrix (necessary to compute interference with other robots)
	 */
	
	public abstract double evaluateInterferenceCost(int robotID ,int taskID,int pathID,double [][][] assignmentMatrix);
	
	
	/**
	 * Evaluate the overall B function, that is the function that consider all interference free costs (i.e. costs related to single robot).
	 */
	protected abstract double [][][] evaluateBFunction();
	
	
	
	/**
	 * Get the number of feasible solution for this optimization problem.
	 * @return The number of feasible solution 
	 */
	
	public int numberOfFeasibleSolutions(){	
		/*
		//Create an optimization problem
		MPSolver optimizationProblemCopy = buildOptimizationProblem(numRobotAug,numTaskAug,maxNumPaths);
		//Solve the optimization problem
	    MPSolver.ResultStatus resultStatus = optimizationProblemCopy.solve();
	    int numberFeasibleSolution = 0;
	    while(resultStatus != MPSolver.ResultStatus.INFEASIBLE ) {
			//Solve the optimization Problem
    		resultStatus = optimizationProblemCopy.solve();
    		//If The solution is feasible increment the number of feasible solution
    		if (resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
    			numberFeasibleSolution = numberFeasibleSolution + 1;
    		}
    		double [][][] assignmentMatrix = saveAssignmentMatrix(numRobotAug,numTaskAug,optimizationProblemCopy);
			//Add the constraint to actual solution -> in order to consider this solution as already found  
    		optimizationProblemCopy = constraintOnPreviousSolution(optimizationProblemCopy,assignmentMatrix);
	    }
		//Return the Total number of feasible solution
	    optimizationProblemCopy.clear();
	    */
		if(feasibleSolutions.size() != 0){
			return this.feasibleSolutions.size();
		}
	    return getFeasibleSolutions().size();
	}
	
	/**
	 * Get all the feasible solutions for this optimization problem.
	 * @return The list of all feasible solutions
	 */

	public List <double [][][]> getFeasibleSolutions(){
		//Define the optimization problem
		//return this.feasibleSolutions;
		
		MPSolver optimizationProblemCopy = buildOptimizationProblem();
		
	    
	    //Initialize a set to store all feasible solution
		
	    
	    //List <double [][][]> feasibleSolutions = new ArrayList <double [][][]>();

	    MPSolver.ResultStatus resultStatus = optimizationProblemCopy.solve();

	    
	    while(resultStatus != MPSolver.ResultStatus.INFEASIBLE ) {
			//Solve the optimization Problem
    		resultStatus = optimizationProblemCopy.solve();
    		//If The solution is feasible increment the number of feasible solution
    		
    		double [][][] assignmentMatrix = getAssignmentMatrix(optimizationProblemCopy);
    		
    		if (resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
    			this.feasibleSolutions.add(assignmentMatrix);
    		}
			//Add the constraint to actual solution -> in order to consider this solution as already found  
    		optimizationProblemCopy = constraintOnPreviousSolution(optimizationProblemCopy,assignmentMatrix);
	    }
	    optimizationProblemCopy.clear();
		//Return the set of all Feasible solutions
	    return this.feasibleSolutions;
	    
		}
	

	/**
	 * Build the optimization problem. User need to define a decision variable as a binary variable and the constraints associated to the problem.
	 * @return A constrained optimization problem
	 */
	protected abstract MPSolver buildOptimizationProblem();
	

	/** 
	 * Find the optimal assignment using the specific algorithm. 
	 * @param optimizationSolver -> An instance of a {@link AbstractOptimizationAlgorithm}
	 * @return The optimal assignment
	 */
	
	protected double [][][] findOptimalAssignment(AbstractOptimizationAlgorithm optimizationSolver){
		this.optimalAssignment = optimizationSolver.solveOptimizationProblem(this);
		this.ScenarioAllocation = null;
		this.scenario = null;
		return this.optimalAssignment;
	}
	
	/**
	 * Get the model of the optimization function (mathematical model).
	 * @return The model of this optimization problem.
	 */
	
	public MPSolver getModel() {
		return this.optimizationModel;
	}
	
	/**
	 * Get the information for this optimization problem.
	 */
	
	protected void getProblemInfo() {
		metaCSPLogger.info("Number of Robot : " + numberRobots);
		metaCSPLogger.info("Number of Task : " + numberTasks);
		metaCSPLogger.info("Number of dummy Robot : " + dummyRobot);
		metaCSPLogger.info("Number of dummy Task : " + dummyTask);
		metaCSPLogger.info("Total Number of Robot : " + numRobotAug);
		metaCSPLogger.info("Total Number of Task : " + numTaskAug);
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
								 //removePath(pss.hashCode());
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
	 * @param optimizationSolver -> An optimization method to solve the optimal assignment problem
	 */
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
						optimizationModel = buildOptimizationProblem();
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
	
	
	private static void printLicense() {
		System.out.println("\n"+AbstractOptimizationProblem.TITLE);
		String cpr = AbstractOptimizationProblem.COPYRIGHT;
		for (String cont : AbstractOptimizationProblem.CONTRIBUTORS) cpr += ", " + cont;
		List<String> cprJust = StringUtils.fitWidth(cpr, 77, 0);
		for (String st : cprJust) System.out.println(st);
		System.out.println();
		if (AbstractOptimizationProblem.LICENSE != null) {
			List<String> lic = StringUtils.fitWidth(AbstractOptimizationProblem.PRIVATE_LICENSE, 72, 5);
			for (String st : lic) System.out.println(st);
		}
		else {
			List<String> lic = StringUtils.fitWidth(AbstractOptimizationProblem.PUBLIC_LICENSE, 72, 5);
			for (String st : lic) System.out.println(st);
		}
		System.out.println();
	}
	
	
	
	
	}

