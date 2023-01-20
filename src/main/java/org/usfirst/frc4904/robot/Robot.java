/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.commands.chassis.ChassisMove;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends CommandRobotBase {
    public RobotMap map = new RobotMap();

    @Override
    public void initialize() {
        driverChooser.setDefaultOption(new NathanGain());
        operatorChooser.setDefaultOption(new DefaultOperator());

        RobotMap.Component.navx.zeroYaw();
        RobotMap.Component.chassisTalonEncoders.reset();
        RobotMap.Component.chassisCANCoders.reset();
    }

    @Override
    public void teleopInitialize() {
        teleopCommand = new ChassisMove(RobotMap.Component.chassis, driverChooser.getSelected());
        LogKitten.v(RobotMap.Component.navx.isConnected() ? "NavX Connected" : "NavX Not Connected");
        LogKitten.v(RobotMap.Component.navx.isMagnetometerCalibrated() ? "Magnetometer Calibrated" : "Magnetometer Not Calibrated");
    }

    @Override
    public void teleopExecute() {
        System.out.println("Teleop Execute");
        SmartDashboard.putBoolean("Is Calibrating", RobotMap.Component.navx.isCalibrating());
        SmartDashboard.putNumber("NavX Yaw Angle", RobotMap.Component.navx.getAngle());
        SmartDashboard.putNumber("NavX Pitch Angle", RobotMap.Component.navx.getPitch());
        SmartDashboard.putNumber("NavX Roll Angle", RobotMap.Component.navx.getRoll());
        SmartDashboard.putNumber("Drive Yaw Angle", RobotMap.Component.SplinesDrive.getHeading());
        SmartDashboard.putString("Wheel Speeds", RobotMap.Component.SplinesDrive.getWheelSpeeds().toString());
        SmartDashboard.putString("Pose", RobotMap.Component.SplinesDrive.getPose().toString());
        SmartDashboard.putNumber("Turn Rate", RobotMap.Component.navx.getRate());
    }
    @Override
    public void autonomousExecute() {
    }

    @Override
    public void autonomousInitialize() {
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
