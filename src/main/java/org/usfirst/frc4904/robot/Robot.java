/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import java.util.List;

import com.ctre.phoenix.sensors.SensorTimeBase;

import org.usfirst.frc4904.robot.commands.SimpleSplines;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.standard.CommandRobotBase;
// import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.commands.chassis.ChassisMove;

public class Robot extends CommandRobotBase {
    private RobotMap map = new RobotMap();
    // new CANCoder

    @Override
    public void initialize() {
        driverChooser.addDefault(new NathanGain());
        RobotMap.Component.navx.zeroYaw();
        // autoChooser.addDefault();
    }

    @Override
    public void teleopInitialize() {
        RobotMap.Component.leftWheelEncoder.setPosition(0);
        RobotMap.Component.rightWheelEncoder.setPosition(0);
        teleopCommand = new ChassisMove(RobotMap.Component.chassis, driverChooser.getSelected());
		teleopCommand.start();
    }

    @Override
    public void teleopExecute() {
    }

    @Override
    public void autonomousInitialize() {
        RobotMap.Component.navx.zeroYaw();
        double yaw = RobotMap.Component.navx.getYaw();
        Command autoCommand = new SimpleSplines(RobotMap.Component.nikhilChassis, 
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
        new Pose2d(1, 0, Rotation2d.fromDegrees(-90)), 
        List.of(
            new Translation2d(.5, 0)
        ));
        if (autoCommand != null) {
            autoCommand.schedule();
        }
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
    public void alwaysExecute() {
        LogKitten.wtf(RobotMap.Component.nikhilChassis.getHeading());
        LogKitten.wtf(RobotMap.Component.navx.getYaw());
    }

}
