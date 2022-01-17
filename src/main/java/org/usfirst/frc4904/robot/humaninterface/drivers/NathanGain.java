package org.usfirst.frc4904.robot.humaninterface.drivers;

import java.util.List;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.LogKitten.KittenLevel;
import org.usfirst.frc4904.standard.commands.Cancel;
import org.usfirst.frc4904.standard.commands.KittenCommand;
import org.usfirst.frc4904.standard.commands.RunIf;
import org.usfirst.frc4904.standard.commands.chassis.ChassisMove;
import org.usfirst.frc4904.standard.commands.chassis.SimpleSplines;
import org.usfirst.frc4904.standard.commands.solenoid.SolenoidExtend;
import org.usfirst.frc4904.standard.commands.solenoid.SolenoidRetract;
import org.usfirst.frc4904.standard.humaninput.Driver;
import org.usfirst.frc4904.standard.commands.motor.MotorPositionSet;
//added this
import org.usfirst.frc4904.standard.subsystems.motor.Motor;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class NathanGain extends Driver {
	public static final double SPEED_GAIN = 1;
	public static final double SPEED_EXP = 2;
	public static final double TURN_GAIN = 0.55;
	public static final double TURN_EXP = 1;
	public static final double Y_SPEED_SCALE = 1;
	public static final double TURN_SPEED_SCALE = 1;

	public NathanGain() {
		super("NathanGain");
	}

	protected double scaleGain(double input, double gain, double exp) {
		return Math.pow(Math.abs(input), exp) * gain * Math.signum(input);
	}

	@Override
	public void bindCommands() {
		// RobotMap.HumanInput.Driver.xbox.a.whenPressed(new SolenoidExtend(RobotMap.Component.solenoid));
		// RobotMap.HumanInput.Driver.xbox.b.whenPressed(new SolenoidRetract(RobotMap.Component.solenoid));

		RobotMap.HumanInput.Driver.xbox.a.whenPressed(new MotorPositionSet(RobotMap.Component.motor));
		RobotMap.HumanInput.Driver.xbox.b.whenPressed(new MotorPositionSet(RobotMap.Component.motor));
	}

	@Override
	public double getX() {
		return 0;
	}

	@Override
	public double getY() {
		double rawSpeed = RobotMap.HumanInput.Driver.xbox.rt.getX() - RobotMap.HumanInput.Driver.xbox.lt.getX();
		double speed = scaleGain(rawSpeed, NathanGain.SPEED_GAIN, NathanGain.SPEED_EXP) * NathanGain.Y_SPEED_SCALE;
		// return 0.5;
		return speed;
	}

	@Override
	public double getTurnSpeed() {
		double rawTurnSpeed = RobotMap.HumanInput.Driver.xbox.leftStick.getX();
		double turnSpeed = scaleGain(rawTurnSpeed, NathanGain.TURN_GAIN, NathanGain.TURN_EXP)
				* NathanGain.TURN_SPEED_SCALE;
		return turnSpeed;
	}
}