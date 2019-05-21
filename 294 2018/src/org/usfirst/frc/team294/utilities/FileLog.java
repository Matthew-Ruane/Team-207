package org.usfirst.frc.team294.utilities;

import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;

public class FileLog {
	
	private FileWriter fileWriter;
	private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
	private String fileName;

	/**
	 * Creates a new log file called "/home/lvuser/logfile.date.time.txt"
	 */
	public FileLog() {
		this("/home/lvuser/logfile");
	}
	
	/**
	 * Creates a new log file.
	 * @param filename Path and name of log file.  ".date.time.txt" will automatically be added to the end of the file name.
	 */
	public FileLog(String filename) {
		this.fileName = filename;
	}
	
	/**
	 * Writes a message to the log file.  The message will be timestamped.  Does not echo the message to the screen.
	 * @param msg
	 */
	public void writeLog(String msg) {
		if (fileWriter == null) {
			final SimpleDateFormat fileDateFormat = new SimpleDateFormat("yyyy-MM-dd.HH-mm-ss");

			try {
				fileWriter = new FileWriter(fileName + ".f5." + fileDateFormat.format(System.currentTimeMillis()) + ".csv", true);
				fileWriter.write("----------------------------\n");
				fileWriter.write(dateFormat.format(System.currentTimeMillis()) + ",  FileLog open.\n");
				fileWriter.flush();
			} catch (IOException exception) {
				System.out.println("Could not open log file: " + exception);
			}
		}
		try {
			fileWriter.write(dateFormat.format(System.currentTimeMillis()) + ", " + msg + "\n");
			fileWriter.flush();
		} catch (IOException exception) {
		}
	}
	
	/**
	 * Writes a message to the log file.  The message will be timestamped.  Also echos the message to the screen.
	 * @param msg
	 */
	public void writeLogEcho(String msg) {
		writeLog(msg);
		System.out.println("Log: " + msg);
	}
	
	/**
	 * Closes the log file.  All writes after closing the log file will be ignored.
	 */
	public void close() {
		try {
			fileWriter.close();
			fileWriter = null;
		} catch (IOException exception) {
		}
	}
	
	

}
