package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.standard.commands.chassis.ChassisMove;
import org.usfirst.frc4904.standard.custom.CustomPIDSourceType;
import org.usfirst.frc4904.standard.custom.PCMPort;
import org.usfirst.frc4904.standard.custom.controllers.CustomJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomXbox;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CustomPIDController;
import org.usfirst.frc4904.standard.custom.motioncontrollers.MotionController;
import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;
import org.usfirst.frc4904.standard.custom.sensors.CustomCANCoder;
import org.usfirst.frc4904.standard.custom.sensors.EncoderPair;
import org.usfirst.frc4904.standard.custom.sensors.NavX;
import org.usfirst.frc4904.standard.custom.sensors.PDP;
import org.usfirst.frc4904.standard.subsystems.SolenoidSubsystem;
import org.usfirst.frc4904.standard.subsystems.chassis.TankDrive;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;
import org.usfirst.frc4904.standard.subsystems.motor.PositionSensorMotor;

public class RobotMap {
    public static class Port {
        public static class HumanInput {
            public static final int XBOX_CONTROLLER = 0;
        }

        public static class CANMotor {
            public static int RIGHT_DRIVE_A = 2;
            public static int RIGHT_DRIVE_B = 3;
            public static int LEFT_DRIVE_A = 4;
            public static int LEFT_DRIVE_B = 5;
            public static int POSITION_MOTOR = 1; // TODO: check port
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

            public static final double CAN_CODER_DEGREES_PER_TICK = DEGREES_PER_REVOLUTION
                    / Metrics.Encoders.CANCoders.TICKS_PER_REVOLUTION;
            public static final double CAN_CODER_METERS_PER_TICK = WHEEL_CIRCUMFERENCE_METERS
                    / Metrics.Encoders.CANCoders.TICKS_PER_REVOLUTION;
        }

        public static class Encoders {
            public static class TalonEncoders {
                public static final double TICKS_PER_REVOLUTION = 2048.0;
                public static final double REVOLUTIONS_PER_TICK = 1 / TICKS_PER_REVOLUTION;
            }

            public static class CANCoders {
                public static final double TICKS_PER_REVOLUTION = 4096.0;
                public static final double REVOLUTIONS_PER_TICK = 1 / TICKS_PER_REVOLUTION;
            }
        }
    }

    public static class PID {
        public static class PositionMotor {
            public static final int P = -1;
            public static final int I = -1;
            public static final int D = -1;
            public static final int F = -1;
        }
    }

    public static class Component {
        public static PDP pdp;
        public static NavX navx;
        public static Motor rightWheelA;
        public static Motor rightWheelB;
        public static Motor leftWheelA;
        public static Motor leftWheelB;
        public static TankDrive chassis;
        public static CustomPIDController drivePID;
        public static CANTalonFX positionTalon;
        public static CANTalonEncoder positionEncoder;
        public static CustomPIDController positionPID;
        public static PositionSensorMotor positionMotor;
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

        /* General */
        Component.pdp = new PDP();
        Component.navx = new NavX(SerialPort.Port.kMXP);
        /* Drive Train */

        // Wheels
        CANTalonFX leftWheelATalon = new CANTalonFX(Port.CANMotor.LEFT_DRIVE_A);
        CANTalonFX rightWheelATalon = new CANTalonFX(Port.CANMotor.RIGHT_DRIVE_B);

        Component.positionTalon = new CANTalonFX(Port.CANMotor.POSITION_MOTOR);
        Component.positionEncoder = new CANTalonEncoder(Component.positionTalon);
        Component.positionPID = new CustomPIDController(0.0000030, 0, 0, 0.01, Component.positionEncoder);
        Component.positionMotor = new PositionSensorMotor(Component.positionPID, Component.positionTalon);

        Component.rightWheelA = new Motor("rightWheelA", false, rightWheelATalon);
        Component.rightWheelB = new Motor("rightWheelB", false, new CANTalonFX(Port.CANMotor.RIGHT_DRIVE_B));
        Component.leftWheelA = new Motor("leftWheelA", true, leftWheelATalon);
        Component.leftWheelB = new Motor("leftWheelB", true, new CANTalonFX(Port.CANMotor.LEFT_DRIVE_B));

        // General Chassis
        Component.chassis = new TankDrive("Blinky-Chassis", Component.leftWheelA, Component.leftWheelB,
                Component.rightWheelA, Component.rightWheelB);
        Component.chassis.setDefaultCommand(new ChassisMove(Component.chassis, new NathanGain()));
    
    } 
    
}
