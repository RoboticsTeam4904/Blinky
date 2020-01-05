package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.SerialPort;

import org.usfirst.frc4904.robot.subsystems.DriveSubsystem;
import org.usfirst.frc4904.standard.custom.PCMPort;
import org.usfirst.frc4904.standard.custom.controllers.CustomJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomXbox;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonSRX;
import org.usfirst.frc4904.standard.custom.sensors.CANEncoder;
import org.usfirst.frc4904.standard.custom.sensors.EncoderPair;
import org.usfirst.frc4904.standard.custom.sensors.NavX;
import org.usfirst.frc4904.standard.custom.sensors.PDP;
import org.usfirst.frc4904.standard.subsystems.chassis.SolenoidShifters;
import org.usfirst.frc4904.standard.subsystems.chassis.TankDriveShifting;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;

public class RobotMap {
    public static class Port {
        public static class HumanInput {
            public static final int joystick = 0;
			public static final int xboxController = 1;
        }

        public static class CANMotor {
            public static int rightDriveA;
            public static int rightDriveB;
            public static int leftDriveA;
            public static int leftDriveB;
        }

        public static class PWM {
        }

        public static class CAN {
            public static final int leftWheelEncoder = -1; // TODO: Update values
			public static final int rightWheelEncoder = -1;
        }

        public static class Pneumatics {
            public static final PCMPort shifter = new PCMPort(0, -1, -1); // TODO: Get real ports
        }

        public static class Digital {
        }
    }

    public static class Metrics {
        public static class Chassis {
            public static final double TICKS_PER_REVOLUTION = -1; // TODO: CHANGE CONSTS
            public static final double DIAMETER_METERS = -1;
            public static final double CIRCUMFERENCE_METERS = Metrics.Chassis.DIAMETER_METERS * Math.PI;
            public static final double TICKS_PER_METER = Metrics.Chassis.TICKS_PER_REVOLUTION
                    / Metrics.Chassis.CIRCUMFERENCE_METERS;
            public static final double DISTANCE_FRONT_BACK = -1;
            public static final double DISTANCE_SIDE_SIDE = -1;
            public static final double METERS_PER_TICK = Metrics.Chassis.CIRCUMFERENCE_METERS
                    / Metrics.Chassis.TICKS_PER_REVOLUTION;
        }
    }

    public static class PID {
        public static class Drive {
        }

        public static class Turn {
        }
    }

    public static class DriveConstants { // TODO: Define all of these.
		public static final boolean kGyroReversed = false;
		public static final double ksVolts = -1;
		public static final double kvVoltSecondsPerMeter = -1;
		public static final double kaVoltSecondsSquaredPerMeter = -1;
		public static final DifferentialDriveKinematics kDriveKinematics = null;
		public static final double kPDriveVel = 0;
    }

    public static class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 0;
		public static final double kMaxAccelerationMetersPerSecondSquared = 0;
		public static final double kRamseteB = 0;
		public static final double kRamseteZeta = 0;
    }

    public static class Component {
        public static PDP pdp;
        public static NavX navx;
        public static CANEncoder leftWheelEncoder;
        public static CANEncoder rightWheelEncoder;
        public static EncoderPair chassisEncoders;
        public static Motor rightWheelA;
        public static Motor rightWheelB;
        public static Motor leftWheelA;
        public static Motor leftWheelB;
        public static SolenoidShifters shifter;
        public static TankDriveShifting chassis;
        public static DriveSubsystem nikhilChassis;
    }

    public static class Input {
    }

    public static class HumanInput {
        public static class Driver {
            public static CustomXbox xbox;
        }

        public static class Operator {
            public static CustomJoystick joystick;
        }
    }

	public static final TrajectoryConstraint autoVoltageConstraint = null;

    public RobotMap() {
        HumanInput.Driver.xbox = new CustomXbox(Port.HumanInput.xboxController);
		HumanInput.Operator.joystick = new CustomJoystick(Port.HumanInput.joystick);
		HumanInput.Driver.xbox = new CustomXbox(Port.HumanInput.xboxController);
		HumanInput.Driver.xbox.setDeadZone(0.1);
		HumanInput.Operator.joystick = new CustomJoystick(Port.HumanInput.joystick);
		/* General */
		Component.pdp = new PDP();
		Component.navx = new NavX(SerialPort.Port.kMXP); // TODO: Update port
		/* Drive Train */
		// Wheel Encoders
		Component.leftWheelEncoder = new CANEncoder("LeftEncoder", Port.CAN.leftWheelEncoder, Metrics.Chassis.METERS_PER_TICK);
		Component.rightWheelEncoder = new CANEncoder("RightEncoder", Port.CAN.rightWheelEncoder, Metrics.Chassis.METERS_PER_TICK);
		Component.chassisEncoders = new EncoderPair(Component.leftWheelEncoder, Component.rightWheelEncoder); // TODO: Update for cancoders
		// Wheels
		Component.rightWheelA = new Motor("rightWheelA", false, new CANTalonSRX(Port.CANMotor.rightDriveA));
		Component.rightWheelB = new Motor("rightWheelB", false, new CANTalonSRX(Port.CANMotor.rightDriveB));
		Component.leftWheelA = new Motor("leftWheelA", false, new CANTalonSRX(Port.CANMotor.leftDriveA));
		Component.leftWheelB = new Motor("leftWheelB", false, new CANTalonSRX(Port.CANMotor.leftDriveB));
		// Shifter
		Component.shifter = new SolenoidShifters(Port.Pneumatics.shifter.buildDoubleSolenoid());
		// General Chassis
		Component.chassis = new TankDriveShifting("2019-Chassis", Component.leftWheelA, Component.leftWheelB,
            Component.rightWheelA, Component.rightWheelB, Component.shifter);
        Component.nikhilChassis = new DriveSubsystem(Component.chassis, Component.leftWheelEncoder, Component.rightWheelEncoder, Component.navx);
    }
}
