package org.usfirst.frc.team294.utilities;

import java.util.Arrays;

import org.usfirst.frc.team294.robot.Robot;

public class VelocityChecker {
	private double[] histArray;
	private int histArrayIndex = 0;
	
	/**
	 * Creates VelocityChecker object to return the running average velocity for the last X seconds.
	 * Note 1:  If less than X secs have elapsed, then the average velocity with be erroneously high.
	 * Note 2:  addValue(velocity) must be called every 20ms for this to work correctly.
	 * @param secs Length of time to compute the running average.
	 */
	public VelocityChecker(double secs) {
		histArray = new double[(int)(secs*50)];
		clearHistory();
	}

	private VelocityChecker() {}

	/**
	 * Creates VelocityChecker object to return the running average velocity for the last N samples.
	 * Note 1:  If fewer than N samples have been added, then the average velocity with be erroneously high.
	 * Note 2:  addValue(velocity) must be called to add samples.
	 * @param size the number of elements in the array
	 */
	public static VelocityChecker getBySize(int size) {
		VelocityChecker temp = new VelocityChecker();
		temp.histArray = new double[size];
		temp.clearHistory();
		return temp;
	}

	/**
	 * Adds a velocity data point to the running average.
	 * @param val velocity data point to add to running average
	 */
	public void addValue(double val) {
		histArray[histArrayIndex] = val;
		histArrayIndex = ++histArrayIndex % histArray.length;
	}

	/**
	 * Clears the running average history.
	 */
	public void clearHistory() {
		Arrays.fill(histArray, Double.MAX_VALUE); // Dont forget, histArray is filled with big numbers to skew avergae
													// at start
		histArrayIndex = 0;
	}
	
	/**
	 * Dumps the HistArray to standard out and the file log
	 */
	public void dumpArray() {
		Robot.log.writeLog("Hist Array dump: "+Arrays.toString(histArray));
	}

	/**
	 * Returns the current running average.
	 * Note 1:  If less than N samples have been added, then the average velocity with be erroneously high.
	 * @return running average
	 */
	public double getAverage() {
		double sum = 0;
		for (double d : histArray) {
//			System.out.print(d + ","); 
			sum += d;
		}
//		System.out.println("avg = " + sum/histArray.length);
		return sum / histArray.length;
	}
}
