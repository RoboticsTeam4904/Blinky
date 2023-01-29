package org.usfirst.frc4904.robot.commands;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.usfirst.frc4904.robot.RobotMap;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RamseteCommandDebug extends RamseteCommand{
    double elapsed = 0; 

    public RamseteCommandDebug(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController follower,
            DifferentialDriveKinematics kinematics, BiConsumer<Double, Double> outputMetersPerSecond,
            Subsystem[] requirements) {
        super(trajectory, pose, follower, kinematics, outputMetersPerSecond, requirements);
    }
    public void initialize() {
        super.initialize();
        ScheduledExecutorService logger = Executors.newScheduledThreadPool(1);
        logger.scheduleAtFixedRate(() -> {
            DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(RobotMap.Component.driveConst.kTrackwidthMeters);
            var wheel_speeds = RobotMap.Component.SplinesDrive.getWheelSpeeds();
            var chassis_speeds = kinematics.toChassisSpeeds(wheel_speeds);
            var actual_velocity = chassis_speeds.vxMetersPerSecond;
            var actual_curvature = chassis_speeds.omegaRadiansPerSecond/actual_velocity;
            var actual_x = RobotMap.Component.SplinesDrive.getPose().getX();
            var actual_y = RobotMap.Component.SplinesDrive.getPose().getY();
            ArrayList<ArrayList> actualdata = new ArrayList<ArrayList>();
            actualdata.add(new ArrayList<Double>(Arrays.asList(elapsed, actual_velocity, actual_x, actual_y)));
            elapsed += 0.02;
        }, 0, 20, TimeUnit.MILLISECONDS);
    }
    public void execute() {
        super.execute();
    }
    public void end(boolean interrupted) {
        super.end(interrupted);
        //logger.shutdownNow();
    }
    public boolean isFinished() {
        return super.isFinished();
    }   
}
