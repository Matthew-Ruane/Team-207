package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.ClawSetMotorSpeed;
import org.usfirst.frc.team294.robot.commands.CubeLetGo;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoSwitchShoot extends CommandGroup {

    public AutoSwitchShoot() {
    	addSequential(new ClawSetMotorSpeed(RobotMap.clawPercentSwitchShoot));
		addSequential(new WaitCommand(.1));
		addSequential(new ClawSetMotorSpeed(RobotMap.clawPercentLetGo));
		addSequential(new WaitCommand(.15));
		addSequential(new CubeLetGo());
//		addSequential(new ClawSetMotorSpeed(.50));
//		addSequential(new WaitCommand(1));
//		addSequential(new ClawSetMotorSpeed(0));
		
    }
}
