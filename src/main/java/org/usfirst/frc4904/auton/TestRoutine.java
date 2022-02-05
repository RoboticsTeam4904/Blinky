package org.usfirst.frc4904.auton;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.chassis.ChassisConstant;
import org.usfirst.frc4904.standard.commands.chassis.ChassisMoveDistance;
import org.usfirst.frc4904.standard.commands.chassis.ChassisTurn;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestRoutine extends SequentialCommandGroup {
    public TestRoutine() {
        this.addCommands(
            // new ChassisMoveDistance(RobotMap.Component.chassis, 10D, null),
            // new ChassisTurn(RobotMap.Component.chassis, 3.14D/2D, null, null),
            // new ChassisMoveDistance(RobotMap.Component.chassis, 10D, null)
            new ChassisConstant(RobotMap.Component.chassis, 0, 0, 0, 0),
            new MotorConstant(RobotMap.Component.extraMotor, 0.5).withTimeout(4),
            new MotorConstant(RobotMap.Component.extraMotor, 0.0).withTimeout(0),
            new ChassisConstant(RobotMap.Component.chassis, 0, 0.2, 0, 4),
            new ChassisConstant(RobotMap.Component.chassis, 0, 0, 0.3, 4),
            new ChassisConstant(RobotMap.Component.chassis, 0, -0.2, 0, 5),
            new ChassisConstant(RobotMap.Component.chassis, 0, 0.2, 0, 1)
        );
    }
}
