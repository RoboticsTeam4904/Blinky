package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.I2C;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.standard.commands.chassis.ChassisMove;
import org.usfirst.frc4904.standard.custom.CustomPIDSourceType;
import org.usfirst.frc4904.standard.custom.controllers.CustomJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomXbox;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CustomPIDController;
import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;
import org.usfirst.frc4904.standard.custom.sensors.CustomCANCoder;
import org.usfirst.frc4904.standard.custom.sensors.EncoderPair;
import org.usfirst.frc4904.standard.custom.sensors.NavX;
import org.usfirst.frc4904.standard.custom.sensors.IMU;
import org.usfirst.frc4904.standard.custom.sensors.PDP;
import org.usfirst.frc4904.standard.subsystems.chassis.TankDrive;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;
import org.usfirst.frc4904.standard.subsystems.chassis.SplinesDrive;
import org.usfirst.frc4904.standard.commands.chassis.SimpleSplines;
import org.usfirst.frc4904.standard.commands.chassis.SimpleSplines.AutoConstants;
import org.usfirst.frc4904.standard.commands.chassis.SimpleSplines.DriveConstants;
import edu.wpi.first.math.geometry.Pose2d;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.AHRSProtocol.AHRSUpdateBase;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;

import edu.wpi.first.wpilibj.SerialPort;

public class RobotMap {
    public static class Port {
        public static class HumanInput {
            public static final int JOYSTICK = 0;
            public static final int XBOX_CONTROLLER = 1;
        }

        public static class CANMotor {
            public static int RIGHT_DRIVE_A = 2;
            public static int RIGHT_DRIVE_B = 4;
            public static int LEFT_DRIVE_A = 3;
            public static int LEFT_DRIVE_B = 5;
        }

        public static class PWM {
        }

        public static class CAN {
            public static final int LEFT_WHEEL_ENCODER = 7;
            public static final int RIGHT_WHEEL_ENCODER = 6;
        }

        public static class Pneumatics {
        }

        public static class Digital {
        }
    }

    public static class Metrics {
        public static class Chassis {
            public static final double DIAMETER_METERS = 0.1016;
            public static final double DISTANCE_SIDE_SIDE = 0.5588;
            public static final double DISTANCE_FRONT_BACK = 0.6604;
            public static final double WHEEL_CIRCUMFERENCE_METERS = Metrics.Chassis.DIAMETER_METERS * Math.PI;
            public static final double DEGREES_PER_REVOLUTION = 360.0;
            public static final double GEAR_RATIO = 17.0; //inverted, 1:GearRatio, in this case 1:17
            public static final double CAN_CODER_DEGREES_PER_TICK = DEGREES_PER_REVOLUTION
                    / Metrics.Encoders.CANCoders.TICKS_PER_REVOLUTION;
            public static final double CAN_CODER_METERS_PER_TICK = WHEEL_CIRCUMFERENCE_METERS
                    / Metrics.Encoders.CANCoders.TICKS_PER_REVOLUTION;
        }

        public static class Encoders {
            public static class TalonEncoders {
                public static final double TICKS_PER_REVOLUTION = 2048.0;
                public static final double REVOLUTIONS_PER_TICK = 1 / TICKS_PER_REVOLUTION;
                public static final double DISTANCE_PER_PULSE = Metrics.Chassis.WHEEL_CIRCUMFERENCE_METERS
                    / TICKS_PER_REVOLUTION / Metrics.Chassis.GEAR_RATIO;
            }

            public static class CANCoders {
                public static final double TICKS_PER_REVOLUTION = 4096.0;
                public static final double REVOLUTIONS_PER_TICK = 1 / TICKS_PER_REVOLUTION;
            }
        }
    }

    public static class Component {
        public static PDP pdp;
        public static NavX navx;
        public static CANTalonEncoder leftWheelTalonEncoder;
        public static CANTalonEncoder rightWheelTalonEncoder;
        public static CustomCANCoder leftWheelCANCoder;
        public static CustomCANCoder rightWheelCANCoder;
        public static EncoderPair chassisTalonEncoders;
        public static EncoderPair chassisCANCoders;
        public static Motor rightWheelA;
        public static Motor rightWheelB;
        public static Motor leftWheelA;
        public static Motor leftWheelB;
        public static TankDrive chassis;
        public static SplinesDrive SplinesDrive;
        public static AutoConstants splineConst;
        public static DriveConstants driveConst;
        public static NavX gyro;
        public static CustomPIDController drivePID;
        public static Pose2d initialPose;
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
        HumanInput.Driver.xbox = new CustomXbox(Port.HumanInput.XBOX_CONTROLLER);
        HumanInput.Driver.xbox.setDeadZone(0.0);
        HumanInput.Operator.joystick = new CustomJoystick(Port.HumanInput.JOYSTICK);

        /* General */
        Component.pdp = new PDP();
        Component.navx = new NavX(I2C.Port.kMXP);
        //testing navx register callback -- not going to work, should comment out


        /* Drive Train */

        // Wheels
        CANTalonFX leftWheelATalon = new CANTalonFX(Port.CANMotor.LEFT_DRIVE_A);
        leftWheelATalon.setNeutralMode(NeutralMode.Brake);
        CANTalonFX rightWheelATalon = new CANTalonFX(Port.CANMotor.RIGHT_DRIVE_A);
        rightWheelATalon.setNeutralMode(NeutralMode.Brake);
        CANTalonFX leftWheelBTalon = new CANTalonFX(Port.CANMotor.LEFT_DRIVE_B);
        leftWheelBTalon.setNeutralMode(NeutralMode.Brake);
        CANTalonFX rightWheelBTalon = new CANTalonFX(Port.CANMotor.RIGHT_DRIVE_B);
        rightWheelBTalon.setNeutralMode(NeutralMode.Brake);


        Component.rightWheelA = new Motor("rightWheelA", false, rightWheelATalon);
        Component.rightWheelB = new Motor("rightWheelB", false, rightWheelBTalon);
        Component.leftWheelA = new Motor("leftWheelA", true, leftWheelATalon);
        Component.leftWheelB = new Motor("leftWheelB", true, leftWheelBTalon);

        // Wheel Encoders
        Component.leftWheelTalonEncoder = new CANTalonEncoder("Leftwheel", leftWheelATalon, true,
        Metrics.Encoders.TalonEncoders.DISTANCE_PER_PULSE, CustomPIDSourceType.kDisplacement,
                FeedbackDevice.IntegratedSensor);
        Component.rightWheelTalonEncoder = new CANTalonEncoder("rightWheel", rightWheelATalon, false,
        Metrics.Encoders.TalonEncoders.DISTANCE_PER_PULSE, CustomPIDSourceType.kDisplacement,
                FeedbackDevice.IntegratedSensor);
        //set talon encoder distance per pulse
        Component.leftWheelTalonEncoder.setDistancePerPulse(Metrics.Encoders.TalonEncoders.DISTANCE_PER_PULSE);
        Component.rightWheelTalonEncoder.setDistancePerPulse(Metrics.Encoders.TalonEncoders.DISTANCE_PER_PULSE);

        Component.leftWheelCANCoder = new CustomCANCoder(Port.CAN.LEFT_WHEEL_ENCODER,
                Metrics.Chassis.CAN_CODER_METERS_PER_TICK);
        Component.rightWheelCANCoder = new CustomCANCoder(Port.CAN.RIGHT_WHEEL_ENCODER,
                Metrics.Chassis.CAN_CODER_METERS_PER_TICK);

        Component.chassisTalonEncoders = new EncoderPair(Component.leftWheelTalonEncoder, Component.rightWheelCANCoder);
        Component.chassisCANCoders = new EncoderPair(Component.leftWheelCANCoder, Component.rightWheelCANCoder);

        // General Chassis
        Component.chassis = new TankDrive("Blinky-Chassis", Component.leftWheelA, Component.leftWheelB,
                Component.rightWheelA, Component.rightWheelB);
        Component.initialPose = new Pose2d(); // TODO double x, double y, rotation2d
        Component.splineConst = new AutoConstants(1.25, 1, 2, 0.7); //need tuning
        Component.driveConst = new DriveConstants(0.44521, 5.7732, 0.45139, 0.50367, 6.5897); //need tuning
        Component.SplinesDrive = new SplinesDrive(Component.chassis, Component.splineConst, Component.driveConst, Component.leftWheelTalonEncoder, Component.rightWheelTalonEncoder, Component.navx, Component.initialPose);

        Component.navx.registerCallback( new ITimestampedDataSubscriber() {
            @Override
            public void timestampedDataReceived(long system_timestamp, long sensor_timestamp, AHRSUpdateBase sensor_data, Object smartdashboard_source) {
                SmartDashboard.putBoolean("Is Calibrating", RobotMap.Component.navx.isCalibrating());
                // SmartDashboard.putNumber("NavX Yaw Angle", RobotMap.Component.navx.getAngle());
                // SmartDashboard.putNumber("NavX Pitch Angle", RobotMap.Component.navx.getPitch());
                // SmartDashboard.putNumber("NavX Roll Angle", RobotMap.Component.navx.getRoll());
                SmartDashboard.putNumber("Drive Yaw Angle", RobotMap.Component.SplinesDrive.getHeading());
                SmartDashboard.putString("Wheel Speeds", RobotMap.Component.SplinesDrive.getWheelSpeeds().toString());
                SmartDashboard.putNumber("Rotation Rate", RobotMap.Component.rightWheelTalonEncoder.getRateSafely());
                SmartDashboard.putString("Pose", RobotMap.Component.SplinesDrive.getPose().toString());
                SmartDashboard.putNumber("Turn Rate", RobotMap.Component.navx.getRate());
            }
        }, SmartDashboard.class);
        ScheduledExecutorService logger = Executors.newScheduledThreadPool(1);
        logger.schedule(() -> {
            var wheel_speeds = RobotMap.Component.SplinesDrive.getWheelSpeeds();
            SmartDashboard.putNumber("Left Wheel Velocity", wheel_speeds.leftMetersPerSecond);
            SmartDashboard.putNumber("Right Wheel Velocity", wheel_speeds.rightMetersPerSecond);
        }, 20, TimeUnit.MILLISECONDS);
        Component.chassis.setDefaultCommand(new ChassisMove(Component.chassis, new NathanGain()));
    }
}
