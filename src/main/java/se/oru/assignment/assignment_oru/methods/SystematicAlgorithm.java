package se.oru.assignment.assignment_oru.methods;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.TreeSet;
import java.util.logging.Logger;

import org.apache.commons.beanutils.BeanUtils;
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
import se.oru.assignment.assignment_oru.OptimizationProblem;
import se.oru.assignment.assignment_oru.Task;
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
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner.PLANNING_ALGORITHM;
import se.oru.coordination.coordination_oru.util.FleetVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.Robot;

import com.google.ortools.linearsolver.*;



public class SystematicAlgorithm extends AbstractOptimizationAlgorithm {
	 
	
	
	
	
	public double [][][] solveOptimizationProblem(MPSolver optimizationProblem,AbstractTrajectoryEnvelopeCoordinator tec,OptimizationProblem oap){
		
		ArrayList<Integer> IDsAllRobots = oap.getRobotsIDs();
		ArrayList<Integer> IDsAllTasks = oap.getTasksIDs();
		
		int numRobotAug = IDsAllRobots.size();
		int numTaskAug = IDsAllTasks.size();
		int maxNumPaths = oap.getmaxNumberOfAlternativePaths();
		double [][][] costValuesMatrix = oap.getCostValuesMatrix();
		
		double linearWeight = oap.getLinearWeight();
	
		
		long timeProva2= 0;
		long timeProvaFinal2= 0;
		
		//Initialize the optimal assignment and the cost associated to it
		double [][][] optimalAssignmentMatrix = new double[numRobotAug][numTaskAug][maxNumPaths];
		double objectiveOptimalValue = 100000000;
		double costBOptimal = 0;
		double costFOptimal = 0;
		
		//Solve the optimization problem

		
		
		MPSolver.ResultStatus resultStatus = optimizationProblem.solve();
		long timeOffsetInitial = Calendar.getInstance().getTimeInMillis();
		long timeOffset = 0;
		int cont = 1;
		while(resultStatus != MPSolver.ResultStatus.INFEASIBLE && timeOffset < oap.getTimeout()) {
			//Evaluate an optimal assignment that minimize only the B function
			timeProva2 = Calendar.getInstance().getTimeInMillis();
			resultStatus = optimizationProblem.solve();
			timeProvaFinal2 = Calendar.getInstance().getTimeInMillis();
			long timeRequiredProva =  timeProvaFinal2-  timeProva2;
			//Evaluate the Assignment Matrix
			double [][][] AssignmentMatrix = oap.saveAssignmentMatrix(numRobotAug,numTaskAug,optimizationProblem);
			//Initialize cost of objective value
			double costofAssignment = 0;
			double costofAssignmentForConstraint = 0;
			double costF = 0;
			//Evaluate the cost of F Function for this Assignment

			double costFFake3 = 0;
			double costBFake3 = 0;
			//Take time to understand how much time require this function
			for (int robotID : IDsAllRobots ) {
				int i = IDsAllRobots.indexOf(robotID);
				for (int taskID : IDsAllTasks ) {
					int j = IDsAllTasks.indexOf(taskID);
					for(int s = 0; s < maxNumPaths; s++) {
						if ( AssignmentMatrix[i][j][s] > 0) {
							
							if (linearWeight != 1) {
								//Evaluate cost of F function only if alpha is not equal to 1
								double costB = optimizationProblem.objective().getCoefficient(optimizationProblem.variables()[i*numTaskAug*maxNumPaths+j*maxNumPaths+s]);
								//fileStream11.println(" ");
								costF = oap.evaluatePathDelay(robotID,taskID,s,AssignmentMatrix,tec)/oap.getSumArrivalTime();
								costofAssignment = linearWeight*costB + (1-linearWeight)*costF + costofAssignment ;
								costofAssignmentForConstraint = costValuesMatrix[i][j][s] + costF + costofAssignmentForConstraint;
								
								costFFake3 = (1-linearWeight)*costF + costFFake3;
								costBFake3 = linearWeight*costB + costBFake3;
								
								
							}
							else {
								double costB = optimizationProblem.objective().getCoefficient(optimizationProblem.variables()[i*numTaskAug*maxNumPaths+j*maxNumPaths+s]);
								costofAssignment = Math.pow(linearWeight*costB, 2) + costofAssignment ;
								costofAssignmentForConstraint = costValuesMatrix[i][j][s]  + costofAssignmentForConstraint;
							}
							

						}
					}
									
				}		
			}		
			
			//Compare actual solution and optimal solution finds so far
			if (costofAssignment < objectiveOptimalValue && resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
				objectiveOptimalValue = costofAssignment;
				for(int i=0; i< AssignmentMatrix.length;i ++) {
					for(int j = 0 ; j <AssignmentMatrix[0].length; j++) {
						for(int s = 0; s < maxNumPaths; s++) {
							optimalAssignmentMatrix[i][j][s] = AssignmentMatrix[i][j][s];
	
						}
					}
				}
				costBOptimal = optimizationProblem.objective().value();
				costFOptimal =  costofAssignmentForConstraint- costBOptimal;
			}
			
			//Add the constraint on cost for next solution
			//add +0,005 in order for tolerance
			optimizationProblem = oap.constraintOnCostSolution(optimizationProblem,costofAssignmentForConstraint);
			//Add the constraint to actual solution in order to consider this solution as already found  
			optimizationProblem = oap.constraintOnPreviousSolution(optimizationProblem,AssignmentMatrix);
			long timeOffsetFinal = Calendar.getInstance().getTimeInMillis();
			timeOffset = timeOffsetFinal - timeOffsetInitial;
			cont +=1;			
		}

		long timeFinal = Calendar.getInstance().getTimeInMillis();
		//long timeRequired = timeFinal- initialTime;
		
		//Return the Optimal Assignment Matrix 
		//String ppMatrixOptimal = "Test"+b+"/Sys/MatrixOptimal-T" + a +".txt";
		//saveMatrixinFile(ppMatrixOptimal,optimalAssignmentMatrix);
		return  optimalAssignmentMatrix;    
	}

	
		
	/** 
	 * Solve the optimization problem given as input considering both B and F Functions. The objective function is defined as sum(c_ij * x_ij) for (i = 1...n)(j = 1...m).
	 * with n = number of robot and m = number of tasks. The solver first finds the optimal solution considering only B function and then
	 * for this each solution (that is an assignment) evaluates the cost of F function. Then a new optimal solution considering only B is 
	 * computed and it is consider only if the cost of this new assignment considering only B is less than the min cost of previous assignments
	 * considering both F and B function
	 * @param optimizationProblem -> An optimization problem defined with {@link #createOptimizationProblem}
	 * @param tec -> an Abstract Trajectory Envelope Coordinator
	 * @param oap -> an optimization problem defined with {@link }

	 * @return An Optimal Assignment that minimize the objective function
	 */	
	public double [][][] solveOptimizationProblem2(MPSolver optimizationProblem,AbstractTrajectoryEnvelopeCoordinator tec,OptimizationProblem oap){
		

		ArrayList<Integer> IDsAllRobots = oap.getRobotsIDs();
		ArrayList<Integer> IDsAllTasks = oap.getTasksIDs();
		
		int numRobotAug = IDsAllRobots.size();
		int numTaskAug = IDsAllTasks.size();
		int maxNumPaths = oap.getmaxNumberOfAlternativePaths();
		double [][][] costValuesMatrix = oap.getCostValuesMatrix();
		
		double linearWeight = oap.getLinearWeight();
		
		long timeToFindASolution = 0;
		//Initialize the optimal assignment and the cost associated to it
		double [][][] optimalAssignmentMatrix = new double[numRobotAug][numTaskAug][maxNumPaths];
		double [][][] optimalAssignmentCostMatrix = new double[numRobotAug][numTaskAug][maxNumPaths];
		double objectiveOptimalValue = 100000000;
		double costBOptimal = 0;
		double costFOptimal = 0;
		
		//Solve the optimization problem	
		MPSolver.ResultStatus resultStatus = optimizationProblem.solve();
		long timeOffsetInitial = Calendar.getInstance().getTimeInMillis();
		long timeOffset = 0;
		int cont = 1;
		while(resultStatus != MPSolver.ResultStatus.INFEASIBLE && timeOffset < oap.getTimeout()) {
			//Evaluate the Assignment Matrix
			double [][][] AssignmentMatrix = oap.saveAssignmentMatrix(numRobotAug,numTaskAug,optimizationProblem);
			//Initialize cost of objective value
			double costofAssignment = 0;
			double costofAssignmentForConstraint = 0;
			double costF = 0;
			//Evaluate the cost of F Function for this Assignment
			double costFFake3 = 0;
			double costBFake3 = 0;
			//Take time to understand how much time require this function
			for (int robotID : IDsAllRobots ) {
				int i = IDsAllRobots.indexOf(robotID);
				for (int taskID : IDsAllTasks ) {
					int j = IDsAllTasks.indexOf(taskID);
					for(int s = 0; s < maxNumPaths; s++) {
						if ( AssignmentMatrix[i][j][s] > 0) {
							
							if (linearWeight != 1) {
								//Evaluate cost of F function only if alpha is not equal to 1
								double costB = optimizationProblem.objective().getCoefficient(optimizationProblem.variables()[i*numTaskAug*maxNumPaths+j*maxNumPaths+s]);
								//fileStream11.println(" ");
								costF = oap.evaluatePathDelay(robotID,taskID,s,AssignmentMatrix,tec)/oap.getSumArrivalTime();
								costofAssignment = linearWeight*costB + (1-linearWeight)*costF + costofAssignment ;
								costofAssignmentForConstraint = costValuesMatrix[i][j][s] + costF + costofAssignmentForConstraint;
								
								costFFake3 = (1-linearWeight)*costF + costFFake3;
								costBFake3 = linearWeight*costB + costBFake3;	
								optimalAssignmentCostMatrix[i][j][s] = costFFake3 + costBFake3;
							}
							else {
								double costB = optimizationProblem.objective().getCoefficient(optimizationProblem.variables()[i*numTaskAug*maxNumPaths+j*maxNumPaths+s]);
								costofAssignment = Math.pow(linearWeight*costB, 2) + costofAssignment ;
								costofAssignmentForConstraint = costValuesMatrix[i][j][s]  + costofAssignmentForConstraint;
								optimalAssignmentCostMatrix[i][j][s] = costofAssignment;
							}
							

						}
					}
									
				}		
			}		
			
			//Compare actual solution and optimal solution finds so far
			if (costofAssignment < objectiveOptimalValue && resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
				objectiveOptimalValue = costofAssignment;
				for(int i=0; i< AssignmentMatrix.length;i ++) {
					for(int j = 0 ; j <AssignmentMatrix[0].length; j++) {
						for(int s = 0; s < maxNumPaths; s++) {
							optimalAssignmentMatrix[i][j][s] = AssignmentMatrix[i][j][s];
	
						}
					}
				}
				costBOptimal = optimizationProblem.objective().value();
				costFOptimal =  costofAssignmentForConstraint- costBOptimal;
			}
			
			//Add the constraint on cost for next solution
			//add +0,005 in order for tolerance
			optimizationProblem = oap.constraintOnCostSolution(optimizationProblem,costofAssignmentForConstraint);
			//Add the constraint to actual solution in order to consider this solution as already found  
			optimizationProblem = oap.constraintOnPreviousSolution(optimizationProblem,AssignmentMatrix);
			long timeOffsetFinal = Calendar.getInstance().getTimeInMillis();
			timeOffset = timeOffsetFinal - timeOffsetInitial;
			cont +=1;			
		}
		
		//Return the Optimal Assignment Matrix in a txt file 
		//String ppMatrixOptimal = "OptimalAssignment.txt";
		//writeMatrix(ppMatrixOptimal,optimalAssignmentMatrix);
		
		return  optimalAssignmentMatrix;    
	}

	
	
	}

