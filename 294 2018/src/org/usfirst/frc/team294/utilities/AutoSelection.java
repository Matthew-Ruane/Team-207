package org.usfirst.frc.team294.utilities;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.commands.autoroutines.*;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelection {
	private boolean allianceSwitchLeft = false;
	private boolean scaleLeft = false;
	private boolean opponentSwitchLeft = false;

	public Command autonomousCommand;

	// Auto path selections
	public static final int AUTO_PLANS = 9;

	public enum AutoPlan {
		ClosestSwitchScale_FFScale, ClosestSwitchScale_FFSwitchFront, ClosestSwitchScale_FFSwitchBack, ScaleOnly, SwitchOnly, BaselineOnly, SameSideSwitchOrBaseline, DeadReckoningForward, DeadReckoningBackward;
		// do not change the order of these
	}

	// Auto field layouts
	public static final int AUTO_FIELD_LAYOUTS = 4;

	public enum AutoFieldLayout {
		LL, LR, RL, RR;
		// first letter is closest switch, second is scale
	}

	// Starting positions
	public enum StartingPosition {
		Left, Middle, Right;
	}

	// Columns in Array are in order of LL, LR, RL, RR
	public static int[][] startingLeftAutoPrograms = {
			{ 1, 3, 1, 2 }, // Plan 0, ClosestSwitchScale_FFScale
			{ 1, 3, 1, 6 }, // Plan 1, ClosestSwitchScale_FFSwitchFront
			{ 1, 3, 1, 4 }, // Plan 2, ClosestSwitchScale_FFSwitchBack
			{ 1, 2, 1, 2 }, // Plan 3, ScaleOnly
			{ 5, 5, 5, 5 }, // Plan 4, SwitchOnly
			{ 7, 7, 7, 7 }, // Plan 5, BaselineOnly
			{ 3, 3, 7, 7 }  //Plan 6, SameSwitchOrBaseLine
	};

	public static int[][] startingMiddleAutoPrograms = {
			{ 5, 5, 5, 5 }, // Plan 0, ClosestSwitchScale_FFScale
			{ 5, 5, 5, 5 }, // Plan 1, ClosestSwitchScale_FFSwitchFront
			{ 5, 5, 5, 5 }, // Plan 2, ClosestSwitchScale_FFSwitchBack
			{ 5, 5, 5, 5 }, // Plan 3, ScaleOnly
			{ 5, 5, 5, 5 }, // Plan 4, SwitchOnly
			{ 7, 7, 7, 7 },  // Plan 5, BaselineOnly
			{ 5, 5, 5, 5 }  //Plan 6, SameSwitchOrBaseLine
	};

	public static int[][] startingRightAutoPrograms = {
			{ 2, 1, 3, 1 }, // Plan 0, ClosestSwitchScale_FFScale
			{ 6, 1, 3, 1 }, // Plan 1, ClosestSwitchScale_FFSwitchFront
			{ 4, 1, 3, 1 }, // Plan 2, ClosestSwitchScale_FFSwitchBack
			{ 2, 1, 2, 1 }, // Plan 3, ScaleOnly
			{ 5, 5, 5, 5 }, // Plan 4, SwitchOnly
			{ 7, 7, 7, 7 },  // Plan 5, BaselineOnly
			{ 7, 7, 3, 3 }  //Plan 6, SameSwitchOrBaseLine
	};

	public AutoSelection() {
		super();
	}
	
	public void readGameData() {
		int programSelected;
		int fieldLayout;
		AutoPlan autoPlan;
		
		Timer timeSinceAutoStart = new Timer();
		timeSinceAutoStart.start();
		
		StartingPosition startPosition = Robot.oi.readStartPosition();
		autoPlan = Robot.oi.readAutoPlan();
		
		if(autoPlan == AutoPlan.DeadReckoningBackward) {
			autonomousCommand = new AutoDeadReckoningBaseline(false);
			Robot.log.writeLogEcho("Ran Dead Reckoning Baseline Backward, side = " + startPosition.name());
			return;
		}else if (autoPlan == AutoPlan.DeadReckoningForward) {
			autonomousCommand = new AutoDeadReckoningBaseline(true);
			Robot.log.writeLogEcho("Ran Dead Reckoning Baseline Forward, side = " + startPosition.name());
			return;
		}
		
		// Retry reading game data until we get a valid string, or 4 seconds have passed.
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		while (gameData.length() < 2 && timeSinceAutoStart.get() < 4) {
			try {
				Thread.sleep(5);
			} catch (InterruptedException ie) {
			}
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}

		if (gameData.length() < 2) {
			// If we still don't have valid game data, then just cross the baseline
			autonomousCommand = new AutoPath7_Baseline(startPosition);
			programSelected = 7;
			fieldLayout = -9999;
			Robot.log.writeLogEcho("Could not read field information from FMS");
			Robot.log.writeLogEcho("Ran Auto Path 7 (Go to baseline), side = " + startPosition.name());

		} else {
			// We have valid game data from the FMS!

			if (gameData.charAt(0) == 'L') {
				SmartDashboard.putBoolean("Close Switch Left", true);
				SmartDashboard.putBoolean("Close Switch Right", false);
				allianceSwitchLeft = true;
			} else {
				SmartDashboard.putBoolean("Close Switch Right", true);
				SmartDashboard.putBoolean("Close Switch Left", false);
				allianceSwitchLeft = false;
			}

			if (gameData.charAt(1) == 'L') {
				SmartDashboard.putBoolean("Scale Left", true);
				SmartDashboard.putBoolean("Scale Right", false);
				scaleLeft = true;
			} else {
				SmartDashboard.putBoolean("Scale Right", true);
				SmartDashboard.putBoolean("Scale Left", false);
				scaleLeft = false;
			}

			if (gameData.charAt(2) == 'L') {
				SmartDashboard.putBoolean("Far Switch Left", true);
				SmartDashboard.putBoolean("Far Switch Right", false);
				opponentSwitchLeft = true;
			} else {
				SmartDashboard.putBoolean("Far Switch Right", true);
				SmartDashboard.putBoolean("Far Switch Left", false);
				opponentSwitchLeft = false;
			}

			if (gameData.startsWith("LL"))
				fieldLayout = AutoFieldLayout.LL.ordinal();
			else if (gameData.startsWith("LR"))
				fieldLayout = AutoFieldLayout.LR.ordinal();
			else if (gameData.startsWith("RL"))
				fieldLayout = AutoFieldLayout.RL.ordinal();
			else
				fieldLayout = AutoFieldLayout.RR.ordinal();

			if (startPosition == StartingPosition.Left) {
				programSelected = startingLeftAutoPrograms[autoPlan.ordinal()][fieldLayout];
			} else if (startPosition == StartingPosition.Middle) {
				programSelected = startingMiddleAutoPrograms[autoPlan.ordinal()][fieldLayout];
			} else {
				programSelected = startingRightAutoPrograms[autoPlan.ordinal()][fieldLayout];

			}

			switch (programSelected) {
			case 1:
				autonomousCommand = new AutoPath1_SameSideScale(startPosition);
				Robot.log.writeLogEcho("Ran Auto Path 1 (same side scale), side = " + startPosition.name());
				break;
			case 2:
				autonomousCommand = new AutoPath2_OppositeSideScale(startPosition);
				Robot.log.writeLogEcho("Ran Auto Path 2 (opposite side scale), side = " + startPosition.name());
				break;
			case 3:
				autonomousCommand = new AutoPath3_SameSideSwitch(startPosition);
				Robot.log.writeLogEcho("Ran Auto Path 3 (same side switch), side = " + startPosition.name());
				break;
			case 4:
				autonomousCommand = new AutoPath4_OppositeSideSwitchBack(startPosition);
				Robot.log.writeLogEcho("Ran Auto Path 4 (opposite side switch back), side = " + startPosition.name());
				break;
			case 5:
				autonomousCommand = new AutoPath5_SwitchFromMiddle(allianceSwitchLeft);
				Robot.log.writeLogEcho("Ran Auto Path 5 (switch from middle), left = " + allianceSwitchLeft);
				break;
			case 6:
				autonomousCommand = new AutoPath6_OppositeSideSwitchFront(startPosition);
				Robot.log.writeLogEcho("Ran Auto Path 6 (opposite side switch front), side = " + startPosition.name());
				break;
			case 7:
				autonomousCommand = new AutoPath7_Baseline(startPosition);
				Robot.log.writeLogEcho("Ran Auto Path 7 (Go to baseline), side = " + startPosition.name());
				break;
			}
		}

		SmartDashboard.putString("Auto path", autonomousCommand.getName());
		SmartDashboard.putNumber("Auto program #", programSelected);
		SmartDashboard.putNumber("Auto field selection", fieldLayout);
		SmartDashboard.putString("Auto plan selected", autoPlan.name());
		SmartDashboard.putString("Auto start position", startPosition.name());

		DriverStation.Alliance color = DriverStation.getInstance().getAlliance();

		if (color == DriverStation.Alliance.Blue) {
			SmartDashboard.putBoolean("Alliance Color", true);
		} else {
			SmartDashboard.putBoolean("Alliance Color", false);
		}
	}
}
