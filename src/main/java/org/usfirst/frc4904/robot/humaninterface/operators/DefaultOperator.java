package org.usfirst.frc4904.robot.humaninterface.operators;
import java.util.List;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.humaninput.Operator;
import org.usfirst.frc4904.standard.custom.controllers.CustomJoystick;
// import org.usfirst.frc4904.standard.commands.chassis.SimpleSplines;
// import org.usfirst.frc4904.standard.subsystems.chassis.SplinesDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DefaultOperator extends Operator {
	public DefaultOperator() {
		super("DefaultOperator");
	}

	public DefaultOperator(String name) {
		super(name);
	}

	@Override
	public void bindCommands() {
		// RobotMap.HumanInput.Operator.joystick.button12.whenPressed(new SimpleSplines(RobotMap.Component.SplinesDrive,RobotMap.Component.SplinesDrive.getPose(), List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(3, 0, new Rotation2d(0)),5)); //change max voltage
		// RobotMap.HumanInput.Operator.joystick.button12.whenPressed(new SimpleSplines(RobotMap.Component.SplinesDrive,RobotMap.Component.SplinesDrive.getPose(), List.of(new Translation2d(3, 0)), new Pose2d(5, 0, new Rotation2d(0)), 12)); //change max voltage
	}
}