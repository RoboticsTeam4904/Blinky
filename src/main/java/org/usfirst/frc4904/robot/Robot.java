/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.SensorTimeBase;

import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.robot.commands.SendSplines;

import org.usfirst.frc4904.standard.CommandRobotBase;
// import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;

import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.commands.chassis.ChassisMove;
import org.usfirst.frc4904.standard.commands.chassis.SimpleSplines;
import org.usfirst.frc4904.standard.subsystems.chassis.SensorDrive;

public class Robot extends CommandRobotBase {
    private RobotMap map = new RobotMap();
    private double maxL = 0;
    private double maxR = 0;
    // new CANCoder

    @Override
    public void initialize() {
        driverChooser.setDefaultOption(new NathanGain());
        RobotMap.Component.navx.zeroYaw();
    }

    @Override
    public void teleopInitialize() {
        RobotMap.Component.leftWheelEncoder.setPosition(0);
        RobotMap.Component.rightWheelEncoder.setPosition(0);
        teleopCommand = new ChassisMove(RobotMap.Component.chassis, driverChooser.getSelected());
    }

    @Override
    public void teleopExecute() {

    }

    @Override
    public void autonomousInitialize() {
    }

    @Override
    public void autonomousExecute() {
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
    public void alwaysExecute() {}

}
