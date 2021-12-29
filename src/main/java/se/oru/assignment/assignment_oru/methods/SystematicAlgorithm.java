package se.oru.assignment.assignment_oru.methods;

import java.util.ArrayList;
import java.util.Calendar;
import se.oru.assignment.assignment_oru.OptimizationProblem;
import se.oru.assignment.assignment_oru.AbstractOptimizationProblem;
import com.google.ortools.linearsolver.*;

/**
 * This class provide a systematic algorithm to solve an optimization problem defined with the class {@link se.oru.assignment.assignment_oru.AbstractOptimizationProblem#buildOptimizationProblem}.
 * 
 * @author pofe
 *
 */

public class SystematicAlgorithm extends AbstractOptimizationAlgorithm{
	 
	
	
	/** 
	 * Solve the optimization problem given as input using a systematic algorithm. The objective function is defined as alpha*B + (1-alpha)*F.
	 * The solver first finds the optimal solution considering only B function and then
	 * for each solution (i.e. a sub-optimal assignment) evaluates the cost of F function. Each new sub-optimal solution (i.e. with the cost less that then previous sub-optimal one)
	 * introduces a new constraint on cost for the next solutions. This constraint prunes all solutions that have a cost considering only B higher that the sub-optimal(that consider B + F) 
	 * since none of these solutions can be the optimal one.
	 * @param oap -> an optimization problem defined with {@link se.oru.assignment.assignment_oru.AbstractOptimizationProblem#buildOptimizationProblem}

	 * @return An Optimal Assignment that minimize the objective function
	 */	
	
	public double [][][] solveOptimizationProblem(AbstractOptimizationProblem oap){
		MPSolver optimizationModel = oap.getModel();
		ArrayList<Integer> IDsAllRobots = oap.getRobotsIDs();
		ArrayList<Integer> IDsAllTasks = oap.getTasksIDs();
		
		int numRobotAug = IDsAllRobots.size();
		int numTaskAug = IDsAllTasks.size();
		int maxNumPaths = oap.getmaxNumberOfAlternativePaths();
		double [][][] costValuesMatrix = oap.getInterferenceFreeCostMatrix();
		
		double linearWeight = oap.getLinearWeight();
	
		
		long timeProva2= 0;
		long timeProvaFinal2= 0;
		
		//Initialize the optimal assignment and the cost associated to it
		double [][][] optimalAssignmentMatrix = new double[numRobotAug][numTaskAug][maxNumPaths];
		double objectiveOptimalValue = 100000000;
		double costBOptimal = 0;
		double costFOptimal = 0;
		
		//Solve the optimization problem
		
		
		MPSolver.ResultStatus resultStatus = optimizationModel.solve();
		long timeOffsetInitial = Calendar.getInstance().getTimeInMillis();
		long timeOffset = 0;
		int cont = 1;
		while(resultStatus != MPSolver.ResultStatus.INFEASIBLE && timeOffset < getTimeout()) {
			//Evaluate an optimal assignment that minimize only the B function
			timeProva2 = Calendar.getInstance().getTimeInMillis();
			resultStatus = optimizationModel.solve();
			timeProvaFinal2 = Calendar.getInstance().getTimeInMillis();
			long timeRequiredProva =  timeProvaFinal2-  timeProva2;
			//Evaluate the Assignment Matrix
			double [][][] AssignmentMatrix = oap.getAssignmentMatrix(optimizationModel);
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
								double costB = optimizationModel.objective().getCoefficient(optimizationModel.variables()[i*numTaskAug*maxNumPaths+j*maxNumPaths+s]);
								//fileStream11.println(" ");
								costF = oap.evaluateInterferenceCost(robotID,taskID,s,AssignmentMatrix);
								costofAssignment = linearWeight*costB + (1-linearWeight)*costF + costofAssignment ;
								costofAssignmentForConstraint = costValuesMatrix[i][j][s] + costF + costofAssignmentForConstraint;
								
								costFFake3 = (1-linearWeight)*costF + costFFake3;
								costBFake3 = linearWeight*costB + costBFake3;
								
								
							}
							else {
								double costB = optimizationModel.objective().getCoefficient(optimizationModel.variables()[i*numTaskAug*maxNumPaths+j*maxNumPaths+s]);
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
				costBOptimal = optimizationModel.objective().value();
				costFOptimal =  costofAssignmentForConstraint- costBOptimal;
			}
			
			//Add the constraint on cost for next solution
			//add +0,005 in order for tolerance
			optimizationModel = oap.constraintOnCostSolution(optimizationModel,costofAssignmentForConstraint);
			//Add the constraint to actual solution in order to consider this solution as already found  
			optimizationModel = oap.constraintOnPreviousSolution(optimizationModel,AssignmentMatrix);
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
	 * @param oap -> an optimization problem defined with {@link }

	 * @return An Optimal Assignment that minimize the objective function
	 */	
	public double [][][] solveOptimizationProblem2(MPSolver optimizationProblem,OptimizationProblem oap){
		

		ArrayList<Integer> IDsAllRobots = oap.getRobotsIDs();
		ArrayList<Integer> IDsAllTasks = oap.getTasksIDs();
		
		int numRobotAug = IDsAllRobots.size();
		int numTaskAug = IDsAllTasks.size();
		int maxNumPaths = oap.getmaxNumberOfAlternativePaths();
		double [][][] costValuesMatrix = oap.getInterferenceFreeCostMatrix();
		
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
		while(resultStatus != MPSolver.ResultStatus.INFEASIBLE && timeOffset < getTimeout()) {
			//Evaluate the Assignment Matrix
			double [][][] AssignmentMatrix = oap.getAssignmentMatrix(optimizationProblem);
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
								costF = oap.evaluateInterferenceCost(robotID,taskID,s,AssignmentMatrix);
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

