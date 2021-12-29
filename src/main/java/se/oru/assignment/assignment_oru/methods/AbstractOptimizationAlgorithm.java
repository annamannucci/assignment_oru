package se.oru.assignment.assignment_oru.methods;

import se.oru.assignment.assignment_oru.AbstractOptimizationProblem;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeCoordinator;


import com.google.ortools.linearsolver.*;

/**
 * This class provides a method to compute an optimal task assignment for a fleet of robots and a set of tasks . 
 * An instantiatable {@link AbstractOptimizationAlgorithm} must provide an implementation of the {@link #solveOptimizationProblem} function to compute the optimal solution
 * @author pofe
 *
 */

public abstract class AbstractOptimizationAlgorithm  {
	

	protected double timeOut = Double.POSITIVE_INFINITY;
	
	
	
	/**
	 * Set the timeOut for the optimization algorithm (in minutes). The algorithm will search a solution until this time.
	 * Use number from 0.1 (10 seconds) to 0.6 (60 seconds) for seconds
	 * @param minutes -> timeout value in minutes
	 * 
	 */
	public void setTimeOutinMinutes(double minutes) {
		if(timeOut < 0) {
			throw new Error("Timeout cannot be negative!");
		}
		if(timeOut < 0.6) {
			this.timeOut = minutes*1000;
		}
		else {
			this.timeOut = minutes*60*1000;
		}
		
	}
	
	
	/**
	 * Get the timeout for this optimization algorithm.
	 * @return The timeout set in minutes. 
	 */
	
	public double getTimeout() {
		return this.timeOut;
	}
	
	
	
	/** 
	 * Solve the optimization problem given as input
	 * @param oap -> An optimization problem defined with {@link #buildOptimizationProblem}
	 * @return The optimal Assignment
	 */	
	public abstract double [][][] solveOptimizationProblem(AbstractOptimizationProblem oap);	
	}

