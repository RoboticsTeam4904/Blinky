package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.SerialPort;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.standard.commands.chassis.ChassisMove;
import org.usfirst.frc4904.standard.commands.chassis.SimpleSplines;
import org.usfirst.frc4904.standard.custom.CustomPIDSourceType;
import org.usfirst.frc4904.standard.custom.PCMPort;
import org.usfirst.frc4904.standard.custom.controllers.CustomJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomXbox;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CustomPIDController;
import org.usfirst.frc4904.standard.custom.sensors.CANEncoder;
import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;
import org.usfirst.frc4904.standard.custom.sensors.EncoderPair;
import org.usfirst.frc4904.standard.custom.sensors.NavX;
import org.usfirst.frc4904.standard.custom.sensors.PDP;
import org.usfirst.frc4904.standard.subsystems.chassis.SensorDrive;
import org.usfirst.frc4904.standard.subsystems.chassis.SolenoidShifters;
import org.usfirst.frc4904.standard.subsystems.chassis.TankDrive;
import org.usfirst.frc4904.standard.subsystems.chassis.TankDriveShifting;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;
import org.usfirst.frc4904.standard.subsystems.motor.VelocitySensorMotor;

public class RobotMap {
    public static class Port {
        public static class HumanInput {
            public static final int joystick = 0;
            public static final int xboxController = 1;
        }

        public static class CANMotor {
            public static int rightDriveA = 2;
            public static int rightDriveB = 3;
            public static int leftDriveA = 4;
            public static int leftDriveB = 5;
            public static int testMotor = 8;
        }

        public static class PWM {
        }

        public static class CAN {
            public static final int leftWheelEncoder = 7;
            public static final int rightWheelEncoder = 6;
        }

        public static class Pneumatics {
            public static final PCMPort shifter = new PCMPort(0, -1, -1); // TODO: Get real ports
        }

        public static class Digital {
        }
    }

    public static class Metrics {
        public static class Chassis {
            public static final double DEGREES_PER_TICK = 0.087890625;
            public static final double METERS_PER_TICK = 1.0 / 2048;// Currently not really MPT, more like revolutions
                                                                    // per tickƒ // .1016 * Math.PI * DEGREES_PER_TICK
                                                                    // / 360.0;
            public static final double TICKS_PER_REVOLUTION = 2048.0; // TODO: CHANGE CONSTS
            // public static final double DIAMETER_METERS = -1;
            // public static final double CIRCUMFERENCE_METERS =
            // Metrics.Chassis.DIAMETER_METERS * Math.PI;
            // public static final double TICKS_PER_METER =
            // Metrics.Chassis.TICKS_PER_REVOLUTION
            // / Metrics.Chassis.CIRCUMFERENCE_METERS;
            public static final double DISTANCE_FRONT_BACK = -1;
            public static final double DISTANCE_SIDE_SIDE = -1;
            // public static final double METERS_PER_TICK =
            // Metrics.Chassis.CIRCUMFERENCE_METERS
            // / Metrics.Chassis.TICKS_PER_REVOLUTION;
        }
    }

    public static class PID {
        public static class Drive {
            public static final double P = 0.01;
            public static final double I = 0.0;
            public static final double D = 0.001;
        }

        public static class Turn {
        }
    }

    // public static class DriveConstants { // TODO: Define all of these.
    // // public static final boolean kGyroReversed = true;
    // public static final double ksVolts = 0.847;
    // public static final double kvVoltSecondsPerMeter = 5.66;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.293;
    // public static final double kTrackwidthMeters = 0.608;
    // public static final DifferentialDriveKinematics kDriveKinematics = new
    // DifferentialDriveKinematics(kTrackwidthMeters);;
    // public static final double kPDriveVel = 9.74;
    // }

    // public static class AutoConstants {
    // public static final double kMaxSpeedMetersPerSecond = 5;
    // public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    // public static final double kRamseteB = 2;
    // public static final double kRamseteZeta = 0.7;
    // }
    public static class DriveConstants {
        // Field Carpet
        // public static final double ksVolts = 0.0018;
        // public static final double kvVoltSecondsPerMeter = 4.9;
        // public static final double kaVoltSecondsSquaredPerMeter = 0.184;
        // public static final double kTrackwidthMeters = .61; //0.5842
        // public static final double kPDriveVel = 6.27;
        // School Carpet
        public static final double ksVolts = 0.000665;
        public static final double kvVoltSecondsPerMeter = 4.9;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0718;
        public static final double kTrackwidthMeters = 0.60; // 0.5842 // 0.6063751884752512
        public static final double kPDriveVel = 1.62;
        // public static final DifferentialDriveKinematics kDriveKinematics = new
        // DifferentialDriveKinematics(kTrackwidthMeters);
        // public static final SplineDriveConstants driveConstants = new
        // SplineDriveConstants(ksVolts, kvVoltSecondsPerMeter,
        // kaVoltSecondsSquaredPerMeter, kTrackwidthMeters, kPDriveVel);
    }

    public static class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2.3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        // public static final SplineAutoConstants autoConstants = new
        // SplineAutoConstants(kMaxSpeedMetersPerSecond,
        // kMaxAccelerationMetersPerSecondSquared, kRamseteB, kRamseteZeta);
    }

    public static class Component {
        public static PDP pdp;
        public static NavX navx;
        public static CANTalonEncoder leftWheelEncoder;
        public static CANTalonEncoder rightWheelEncoder;
        public static CANTalonEncoder testEncoder;
        public static VelocitySensorMotor testMotor;
        public static EncoderPair chassisEncoders;
        public static Motor rightWheelA;
        public static Motor rightWheelB;
        public static Motor leftWheelA;
        public static Motor leftWheelB;
        public static SolenoidShifters shifter;
        public static TankDrive chassis;
        public static CANCoderConfiguration _canCoderConfiguration;
        public static SensorDrive nikhilChassis;
        public static CustomPIDController drivePID;
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

    public RobotMap() {
        HumanInput.Driver.xbox = new CustomXbox(Port.HumanInput.xboxController);
        HumanInput.Operator.joystick = new CustomJoystick(Port.HumanInput.joystick);
        HumanInput.Driver.xbox = new CustomXbox(Port.HumanInput.xboxController);
        HumanInput.Driver.xbox.setDeadZone(0.0);
        HumanInput.Operator.joystick = new CustomJoystick(Port.HumanInput.joystick);
        /* General */
        Component.pdp = new PDP();
        Component.navx = new NavX(SerialPort.Port.kMXP);
        /* Drive Train */

        // Wheels
        CANTalonFX leftWheelATalon = new CANTalonFX(Port.CANMotor.rightDriveA);
        CANTalonFX rightWheelATalon = new CANTalonFX(Port.CANMotor.rightDriveA);
        CANTalonFX testTalon = new CANTalonFX(Port.CANMotor.testMotor);
        Component.leftWheelEncoder = new CANTalonEncoder("Leftwheel", leftWheelATalon, true,
                Metrics.Chassis.METERS_PER_TICK, CustomPIDSourceType.kDisplacement, FeedbackDevice.IntegratedSensor);
        Component.rightWheelEncoder = new CANTalonEncoder("rightWheel", rightWheelATalon, true,
                Metrics.Chassis.METERS_PER_TICK, CustomPIDSourceType.kDisplacement, FeedbackDevice.IntegratedSensor);
        Component.testEncoder = new CANTalonEncoder("testMotor", testTalon, false, Metrics.Chassis.METERS_PER_TICK,
                CustomPIDSourceType.kRate, FeedbackDevice.IntegratedSensor);
        Component.drivePID = new CustomPIDController(PID.Drive.P, PID.Drive.I, PID.Drive.D, Component.testEncoder);
        Component.testMotor = new VelocitySensorMotor("testMotor", Component.drivePID, testTalon);
        // Component.rightWheelA = new Motor("rightWheelA", Component.drivePID,
        // rightWheelATalon);
        Component.rightWheelA = new Motor("rightWheelA", false, rightWheelATalon);
        Component.rightWheelB = new Motor("rightWheelB", false, new CANTalonFX(Port.CANMotor.rightDriveB));
        Component.leftWheelA = new Motor("leftWheelA", true, leftWheelATalon);
        Component.leftWheelB = new Motor("leftWheelB", true, new CANTalonFX(Port.CANMotor.leftDriveB));
        // Wheel Encoders

        // Component.leftWheelEncoder = new CANCoder("LeftEncoder",
        // Port.CAN.leftWheelEncoder, Metrics.Chassis.METERS_PER_TICK);
        // Component.rightWheelEncoder = new CANEncoder("RightEncoder",
        // Port.CAN.rightWheelEncoder, Metrics.Chassis.METERS_PER_TICK);
        // Component.leftWheelEncoder = new CANCoder(Port.CAN.leftWheelEncoder);
        // Component.rightWheelEncoder = new CANCoder(Port.CAN.rightWheelEncoder);
        // Component._canCoderConfiguration = new CANCoderConfiguration();
        // Component.leftWheelEncoder.configAllSettings(Component._canCoderConfiguration);
        // Component.rightWheelEncoder.configAllSettings(Component._canCoderConfiguration);
        // Component.leftWheelEncoder.configSensorDirection(true);
        // Component.leftWheelEncoder.configFeedbackCoefficient(RobotMap.Metrics.Chassis.METERS_PER_TICK,
        // "meters", SensorTimeBase.PerSecond);
        // Component.rightWheelEncoder.configFeedbackCoefficient(RobotMap.Metrics.Chassis.METERS_PER_TICK,
        // "meters", SensorTimeBase.PerSecond);
        // Shifter
        // Component.shifter = new
        // SolenoidShifters(Port.Pneumatics.shifter.buildDoubleSolenoid());
        // General Chassis
        Component.chassis = new TankDrive("Blinky-Chassis", Component.leftWheelA, Component.leftWheelB,
                Component.rightWheelA, Component.rightWheelB);
        Component.chassis.setDefaultCommand(new ChassisMove(Component.chassis, new NathanGain()));
    }
}
