package se.oru.assignment.assignment_oru;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.assignment.assignment_oru.util.robotType.ROBOT_TYPE;
import se.oru.coordination.coordination_oru.Mission;


public class Task {
	
	protected int taskID;
	protected PoseSteering startLocation;
	protected PoseSteering goalLocation;
	protected int decidedRobotID = -1;
	protected List<PoseSteering[]> paths = null;
	protected Set<ROBOT_TYPE> robotTypes = null;
	protected double deadline = -1;
	protected double operationTime = 0;
	protected boolean priority = false;
	protected int robotRequired = 1;
	
	
	
	/**
 	 * Constructor. Generate a Task with a Starting Pose and an Ending Pose; the type is used to evaluate which 
	 * robot can perform this task
	 *  @param taskID -> the Id of Task
	 * @param deadline -> deadline of Task 
	 * @param StartPose -> Task Starting Position
	 * @param GoalPose -> Task Ending Position
	 * @param robotTypes -> robot type that can execute this task
	 */
	public Task (int taskID,Pose StartPose, Pose GoalPose, ROBOT_TYPE ... robotTypes) {
		this(taskID,-1,new PoseSteering(StartPose, 0.0), new PoseSteering(GoalPose, 0.0),robotTypes);
	}

	/**
 	 * Constructor. Generate a Task with a Starting Pose and an Ending Pose; the type is used to evaluate which 
	 * robot can perform this task
	 * @param taskID -> the Id of Task
	 * @param deadline -> deadline of Task 
	 * @param StartPose -> Task Starting Position
	 * @param GoalPose -> Task Ending Position
	 * @param robotTypes -> robot type that can execute this task
	 */
	public Task (int taskID,double deadline,Pose StartPose, Pose GoalPose, ROBOT_TYPE ... robotTypes) {
		this(taskID,deadline,new PoseSteering(StartPose, 0.0), new PoseSteering(GoalPose, 0.0),robotTypes);
	}

	
	/**
 	 * Constructor. Generate a Task with a Starting Pose and an Ending Pose; the type is used to evaluate which 
	 * robot can perform this task
	 * @param taskID -> the Id of Task
	 * @param StartPose -> Task Starting Position

	 * @param GoalPose -> Task Ending Position
	 * @param deadline -> deadline of Task
	 * @param robotTypes -> robot type that can execute this task
	 */
	public Task (int taskID,PoseSteering StartPose, PoseSteering GoalPose, ROBOT_TYPE ... robotTypes) {
		this.taskID = taskID;
		this.startLocation = StartPose;
		this.goalLocation = GoalPose;

		if (robotTypes.length == 0) throw new Error("Need to specifiy at least one robot type!");
		this.robotTypes = new HashSet<ROBOT_TYPE>();
		for (ROBOT_TYPE rt : robotTypes) this.robotTypes.add(rt);
	}
	
	/**
 	 * Constructor. Generate a Task with a Starting Pose and an Ending Pose; the type is used to evaluate which 
	 * robot can perform this task
	 * @param taskID -> the Id of Task
	 * @param deadline -> deadline of Task 
	 * @param StartPose -> Task Starting Position
	 * @param GoalPose -> Task Ending Position
	 * @param deadline -> deadline of Task
	 * @param robotTypes -> robot type that can execute this task
	 */
	public Task (int taskID,double deadline, PoseSteering StartPose, PoseSteering GoalPose, ROBOT_TYPE ... robotTypes) {
		this.taskID = taskID;
		this.startLocation = StartPose;
		this.goalLocation = GoalPose;
		this.deadline = deadline;
		if (robotTypes.length == 0) throw new Error("Need to specifiy at least one robot type!");
		this.robotTypes = new HashSet<ROBOT_TYPE>();
		for (ROBOT_TYPE rt : robotTypes) this.robotTypes.add(rt);
	}

	public List<PoseSteering[]> getPaths() {
		return paths;
	}
	
	public void setPaths(PoseSteering[] ... newPaths) {
		if (this.paths == null) this.paths = new ArrayList<PoseSteering[]>();
		for (int i = 0; i < newPaths.length-1; i++) {
			//if (!(newPaths[i][newPaths[i].length-1].equals(newPaths[i+1][0]))) throw new Error("Teletransport not supported yet!");
			if (!(ArrayUtils.isEquals(newPaths[i][newPaths[i].length-1].getPose().toString(),newPaths[i+1][0].getPose().toString()))) throw new Error("Teletransport not supported yet!");
			this.paths.add(newPaths[i]);
		}
		this.paths.add(newPaths[newPaths.length-1]);
	}
	
	public int getNumPaths() {
		if (this.paths == null) return 0;
		return this.paths.size();
	}
	
	public Pose getStartPose() {
		return this.startLocation.getPose();		
	}
	
	public Pose getGoalPose() {
		return this.goalLocation.getPose();	
	}

	public PoseSteering getStart() {
		return this.startLocation;		
	}
	
	public PoseSteering getGoal() {
		return this.goalLocation;	
	}
	
	public boolean isPriority() {
		return this.priority;	
	}

	
	public void setPriority(boolean priority) {
		this.priority = priority;	
	}
	
	public void setRobotRequired(int  robotRequired) {
		this.robotRequired = robotRequired;	
	}
	
	public int getRobotRequired() {
		return this.robotRequired;	
	}
	

	public boolean isCompatible(ROBOT_TYPE robotType) {
		boolean compatible = false;
		
		if (this.robotTypes.contains(robotType)) {
			compatible = true;
		}
		
		return compatible;
	}
	
	public boolean isTaskAssigned() {
		return this.decidedRobotID != -1;
		//return this.taskIsAssigned;
	}
	
	public int isTaskAssignedID() {
		return this.decidedRobotID ;
		//return this.taskIsAssigned;
	}

	public boolean isDeadlineSpecified() {
		return this.deadline != -1;
		//return this.taskIsAssigned;
	}
	
	
	public void setDeadline(double deadline) {
		this.deadline = deadline;
	}
	
	public double getDeadline() {
		return this.deadline;
	}
	
	public int getID() {
		return this.taskID;
	}
	
	public void setOperationTime(double operationTime) {
		this.operationTime = operationTime;
	}
	
	public double getOperationTime() {
		return this.operationTime;
	}
	
	
	public String toString() {
		return "Task " + taskID +" info: " + "From location: " + this.startLocation + " to Goal Location: " + this.goalLocation + (deadline != -1 ? " (deadline: " + deadline + ")" : "");
	}
	
	
	public void getInfo() {
		System.out.println("Starting Pose -> " +this.startLocation + "\n Goal Pose ->"+ this.goalLocation + "\n Robot Types ->"+ this.robotTypes
				+"\n Task is Assigned "+ this.isTaskAssigned());
	}
	
	public void assignRobot(int robotID) {
		this.decidedRobotID = robotID;
	}

	public Mission[] getMissions() {
		if (this.paths == null) throw new Error("No paths specified!");
		if (this.decidedRobotID == -1) throw new Error("No robot assigned!");
		Mission[] ret = new Mission[paths.size()];
		for (int i = 0; i < paths.size(); i++) {
			ret[i] = new Mission(this.decidedRobotID, paths.get(i));
			if (i == 0) ret[i].setFromLocation("Init for Robot" + this.decidedRobotID);
			else if (i ==  paths.size()-1) ret[i].setFromLocation("Goal for Robot" + this.decidedRobotID);
			else ret[i].setFromLocation("Waypoint " + i + " for Robot" + this.decidedRobotID);
		}
		return ret;
		//Now you can do this: Missions.enqueue(task.getMissions());
	}
	
	
	
}
