package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class LoadCubeManual extends CommandGroup {

    public LoadCubeManual() {
    	
    	addSequential(new ClawSetOpen(true));
    	addSequential(new ArmMoveWithPiston(RobotMap.armIntakePos, true));
    	addSequential(new ClawSetMotorSpeed(RobotMap.clawPercentIn));
    }
}
