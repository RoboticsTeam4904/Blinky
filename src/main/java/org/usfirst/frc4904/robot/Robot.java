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
import org.usfirst.frc4904.standard.CommandRobotBase;
// import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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
        // autoChooser.addDefault();
        // RobotMap.Component.rightATalonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PIDIDX, 10);
    }

    @Override
    public void teleopInitialize() {
        LogKitten.wtf(driverChooser.getSelected());
        // RobotMap.Component.leftWheelEncoder.setPosition(0);
        // RobotMap.Component.rightWheelEncoder.setPosition(0);
        teleopCommand = new ChassisMove(RobotMap.Component.chassis, driverChooser.getSelected());
        teleopCommand.schedule();
        
    }

    @Override
    public void teleopExecute() {
    }

    @Override
    public void autonomousInitialize() {
        RobotMap.Component.navx.zeroYaw();
        RobotMap.Component.nikhilChassis = new SensorDrive(RobotMap.Component.chassis, RobotMap.AutoConstants.autoConstants, RobotMap.DriveConstants.driveConstants, RobotMap.Component.leftWheelEncoder, RobotMap.Component.rightWheelEncoder, RobotMap.Component.navx);
        Trajectory traj = RobotMap.Component.nikhilChassis.generateQuinticTrajectory(List.of(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            // new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(Units.feetToMeters(10), Units.feetToMeters(-2), Rotation2d.fromDegrees(0))));
        Command autoCommand = new SimpleSplines(RobotMap.Component.nikhilChassis, traj);
        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }

    @Override
    public void autonomousExecute() {
    }

    @Override
    public void disabledInitialize() {
        // LogKitten.wtf(RobotMap.Component.navx.getYaw());
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
        // LogKitten.wtf(RobotMap.Component.nikhilChassis.getHeading());
        // LogKitten.wtf(RobotMap.Component.navx.getYaw());
        // LogKitten.wtf(RobotMap.Component.nikhilChassis.getPose());
        // LogKitten.wtf(RobotMap.Component.leftWheelEncoder.getVelocity());
        // if (RobotMap.Component.leftWheelEncoder.getVelocity() > maxL){
        //     maxL = RobotMap.Component.leftWheelEncoder.getVelocity();
        // }
        // if (RobotMap.Component.rightWheelEncoder.getVelocity() > maxR){
        //     maxR = RobotMap.Component.rightWheelEncoder.getVelocity();
        // }
        // LogKitten.wtf
        // double magPos = RobotMap.Component.rightWheelEncoder.getPosition();
        // double falcPos = RobotMap.Component.
        // LogKitten.wtf()
        // LogKitten.wtf("Left " + RobotMap.Component.leftWheelEncoder.getPosition());
        // LogKitten.wtf("Right " + RobotMap.Component.rightWheelEncoder.getPosition());
        // LogKitten.wtf(RobotMap.Component.pdp.getVoltage());
        // LogKitten.wtf(RobotMap.Component.navx.getYaw());
    }

}
