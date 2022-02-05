package org.usfirst.frc4904.auton;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MotorRoutine extends SequentialCommandGroup {
    public MotorRoutine() {
        this.addCommands(
            new MotorConstant(RobotMap.Component.extraMotor, 0.5).withTimeout(4)
        );
    }
}
