/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

// import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.commands.DebugTankDriveVolts;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.LogKitten;
// import org.usfirst.frc4904.standard.commands.chassis.ChassisMove;
// import org.usfirst.frc4904.standard.commands.chassis.SimpleSplines;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Robot extends CommandRobotBase {
    // public RobotMap map = new RobotMap();
    private static CANTalonFX ceilingTalon;
    private static double MAX_VOLTAGE = 12; // MAX VOLTAGE for splines


    @Override
    public void initialize() {
        // driverChooser.setDefaultOption(new NathanGain());
        // operatorChooser.setDefaultOption(new DefaultOperator());

        // RobotMap.Component.navx.zeroYaw();
        // RobotMap.Component.chassisTalonEncoders.reset();
        // RobotMap.Component.chassisCANCoders.reset();
        Robot.ceilingTalon = new CANTalonFX(1);
    }

    @Override
    public void teleopInitialize() {
        // teleopCommand = new ChassisMove(RobotMap.Component.chassis, driverChooser.getSelected());
        LogKitten.v(RobotMap.Component.navx.isConnected() ? "NavX Connected" : "NavX Not Connected");
        LogKitten.v(RobotMap.Component.navx.isMagnetometerCalibrated() ? "Magnetometer Calibrated" : "Magnetometer Not Calibrated");
    }

    @Override
    public void teleopExecute() {
        SmartDashboard.putBoolean("Test?", true);
        // SmartDashboard.putBoolean("Is Calibrating", RobotMap.Component.navx.isCalibrating());
        // SmartDashboard.putNumber("NavX Yaw Angle", RobotMap.Component.navx.getAngle());
        // SmartDashboard.putNumber("NavX Pitch Angle", RobotMap.Component.navx.getPitch());
        // SmartDashboard.putNumber("NavX Roll Angle", RobotMap.Component.navx.getRoll());
        // SmartDashboard.putNumber("Drive Yaw Angle", RobotMap.Component.SplinesDrive.getHeading());
        // SmartDashboard.putString("Wheel Speeds", RobotMap.Component.SplinesDrive.getWheelSpeeds().toString());
        // SmartDashboard.putString("Pose", RobotMap.Component.SplinesDrive.getPose().toString());
        // SmartDashboard.putNumber("Turn Rate", RobotMap.Component.navx.getRate());
    }
    @Override
    public void autonomousExecute() {
        double targetPos = 4096;
        LogKitten.wtf("Autonomous execute");
        // TalonFXControlMode
     //   Robot.ceilingTalon.set(TalonFXControlMode.PercentOutput, 0.5);
        Robot.ceilingTalon.set(TalonFXControlMode.MotionMagic, 2048);

    }

    @Override
    public void autonomousInitialize() {
        // SimpleSplines spline = new SimpleSplines(RobotMap.Component.SplinesDrive,
        //     new Pose2d(0,0, new Rotation2d(0)),
        //     List.of(
        //         new Translation2d(1, 0),
        //         new Translation2d(2, 0)
        //         ),
        //     new Pose2d(5, 0, new Rotation2d(0)), 
        // MAX_VOLTAGE);
        // spline.schedule();
        // (new InstantCommand(() -> RobotMap.Component.leftWheelA.set(1))).schedule();
        // DebugTankDriveVolts voltageTest = new DebugTankDriveVolts(RobotMap.Component.chassis, 12, 12);
        // voltageTest.schedule();

        LogKitten.wtf("autonomous initialize");
        final class Constants {
            public static final int pid_slot = 0; // primary pid loop on the falcon (they are cascading)
            public static final int pid_idx = 0; // primary pid loop on the falcon (they are cascading)
            public static final int timeout_ms = 30;
            final class Gains {
                public static final double kP = 0.5;
                public static final double kI = 0;
                public static final double kD = 0;
                public static final double kF = 0;
            }
        }
        Robot.ceilingTalon.configFactoryDefault();  // init commit
        Robot.ceilingTalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.pid_idx, Constants.timeout_ms);
        Robot.ceilingTalon.configNeutralDeadband(0.001, Constants.timeout_ms); // default deadband is 0.04 = 4%, this is 0.1%. thus it should do smt with neutral mode when it strays out of the deadband??
        Robot.ceilingTalon.setSensorPhase(false); Robot.ceilingTalon.setInverted(false); // or something TODO
        // apparently setSensorPhase can be false for the integrated sensor. must be true in other cases??

        // set frame periods to be at least as fast as the periodic rate
        Robot.ceilingTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.timeout_ms);
        Robot.ceilingTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.timeout_ms);

        Robot.ceilingTalon.configNominalOutputForward(0, Constants.timeout_ms);
        Robot.ceilingTalon.configNominalOutputReverse(0, Constants.timeout_ms);
        Robot.ceilingTalon.configPeakOutputForward( 1, Constants.timeout_ms);
        Robot.ceilingTalon.configPeakOutputReverse(-1, Constants.timeout_ms);

        // set configs
        Robot.ceilingTalon.selectProfileSlot(Constants.pid_slot, Constants.pid_idx);
        Robot.ceilingTalon.config_kP(Constants.pid_slot, Constants.Gains.kP, Constants.timeout_ms);
        Robot.ceilingTalon.config_kI(Constants.pid_slot, Constants.Gains.kI, Constants.timeout_ms);
        Robot.ceilingTalon.config_kD(Constants.pid_slot, Constants.Gains.kD, Constants.timeout_ms);
        Robot.ceilingTalon.config_kF(Constants.pid_slot, Constants.Gains.kF, Constants.timeout_ms);

        // set (presumably max?) acceleration and cruise velocity
        Robot.ceilingTalon.configMotionCruiseVelocity(15000, Constants.timeout_ms);
        Robot.ceilingTalon.configMotionAcceleration(6000, Constants.timeout_ms);

        // zero sensor
        Robot.ceilingTalon.setSelectedSensorPosition(0, Constants.pid_idx, Constants.timeout_ms);
    }

    @Override
    public void disabledInitialize() {
    }

    @Override
    public void disabledExecute() {
    }

    @Override
    public void testInitialize() {
    }

    @Override
    public void testExecute() {
    }

    @Override
    public void alwaysExecute() {
    }

}
