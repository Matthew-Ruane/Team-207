/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.WaitCommand;

import frc.robot.commands.SetElevatorShoulderWrist; 
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.BallFeeder;
import frc.robot.subsystems.HatchGroundIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FrontClimber;
import frc.robot.subsystems.HatchPatchSubsystem;
import frc.robot.HAL;

public class Transfer extends CommandGroup {

	public Transfer() {
        requires(HAL.ballFeed);
        requires(HAL.hatchGroundIntake);

        // STEP 1
        addParallel(new SetElevatorShoulderWrist(Wrist.WristPreset.HATCH_GROUND_2_ELEVATOR_TRANSFER, Shoulder.ShoulderPreset.HATCH_GROUND_2_ELEVATOR_TRANSFER, Elevator.ElevatorPreset.HATCH_GROUND_2_ELEVATOR_TRANSFER));
        addSequential(new WaitCommand(1.5));
        //addSequential(HAL.hatch.setStateCommand(HatchPatchSubsystem.State.IN));
        
        // STEP 2
        addParallel(new SetFrontClimber(FrontClimber.FrontClimberPreset.PASS3));
        addSequential(HAL.hatchGroundIntake.setStateCommand(HatchGroundIntake.State.FEED, HatchGroundIntake.State.STOP, 1.5));
        
        // STEP 3
        addSequential(HAL.hatch.setStateCommand(HatchPatchSubsystem.State.OUT));

        // STEP 4
        addParallel(HAL.hatchGroundIntake.setStateCommand(HatchGroundIntake.State.UNFEED, HatchGroundIntake.State.STOP, 2));
        addSequential(new SetElevatorShoulderWrist(Wrist.WristPreset.TRAVEL, Shoulder.ShoulderPreset.TRAVEL, Elevator.ElevatorPreset.TRAVEL));
    
        // STEP 5
        addParallel(new SetFrontClimber(FrontClimber.FrontClimberPreset.HOME));


        //addSequential(HAL.ballFeed.setStateCommand(BallFeeder.State.FEED, BallFeeder.State.STOP, 1));
        //addSequential(HAL.ballFeed.setStateCommand(BallFeeder.State.FEED, BallFeeder.State.STOP, 1));

        end();
	}

    // protected void initialize() {
		
    // }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        //addSequential(new SetElevatorShoulderWrist(Wrist.WristPreset.HATCH_GROUND_2_ELEVATOR_TRANSFER, Shoulder.ShoulderPreset.HATCH_GROUND_2_ELEVATOR_TRANSFER, Elevator.ElevatorPreset.HATCH_GROUND_2_ELEVATOR_TRANSFER));
        //end();
        //addSequential(new SetElevatorShoulderWrist(Wrist.WristPreset.CARGOSHIP_BALL, Shoulder.ShoulderPreset.CARGOSHIP_BALL, Elevator.ElevatorPreset.CARGOSHIP_BALL));

	}

    // Make this return true when this Command no longer needs to run execute()
    // protected boolean isFinished() {
    // 	return false;
    // }

    // Called once after isFinished returns true
    // protected void end() {
    	
    // }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    // protected void interrupted() {
    // }
}