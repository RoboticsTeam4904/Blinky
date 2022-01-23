package org.usfirst.frc4904.auton;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.chassis.ChassisMoveDistance;
import org.usfirst.frc4904.standard.commands.chassis.ChassisTurn;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestRoutine extends SequentialCommandGroup {
    public TestRoutine() {
        this.addCommands(
            new ChassisMoveDistance(RobotMap.Component.chassis, 10D, null),
            new ChassisTurn(RobotMap.Component.chassis, 3.14D/2D, null, null),
            new ChassisMoveDistance(RobotMap.Component.chassis, 10D, null)
        );
    }
}
