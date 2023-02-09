/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import java.util.List;

import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.commands.DebugTankDriveVolts;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.LogKitten;
// import org.usfirst.frc4904.standard.commands.chassis.ChassisMove;
// import org.usfirst.frc4904.standard.commands.chassis.SimpleSplines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Robot extends CommandRobotBase {
    public RobotMap map = new RobotMap();
    private static double MAX_VOLTAGE = 12; // MAX VOLTAGE for splines


    @Override
    public void initialize() {
        // driverChooser.setDefaultOption(new NathanGain());
        // operatorChooser.setDefaultOption(new DefaultOperator());

        RobotMap.Component.navx.zeroYaw();
        RobotMap.Component.chassisTalonEncoders.reset();
        RobotMap.Component.chassisCANCoders.reset();
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
        
        RobotMap.Component.ceilingTalon;
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
