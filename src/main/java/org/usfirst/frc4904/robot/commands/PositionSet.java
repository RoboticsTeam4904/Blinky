package org.usfirst.frc4904.robot.commands;

import org.usfirst.frc4904.robot.RobotMap;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PositionSet extends CommandBase {
    double position;

    public PositionSet(double position) {
        super();
        this.position = position;
    }

    @Override
    public void initialize() {
        RobotMap.Component.positionMotor.setSelectedSensorPosition(this.position);
    }
}
