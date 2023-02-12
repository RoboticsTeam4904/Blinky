package org.usfirst.frc4904.robot.commands;

import java.util.List;

import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.subsystems.chassis.SplinesDrive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TrajectoryTest extends SequentialCommandGroup{
    public TrajectoryTest(SplinesDrive robotDrive, Pose2d init_pos, List<Translation2d> inter_points, Pose2d final_pos){
        long startTime = RobotController.getFPGATime();
        int maxVoltage = 10;
        for (Translation2d point : inter_points) {
            point = point.plus(new Translation2d(init_pos.getX(), init_pos.getY()));
        }
        final_pos = new Pose2d(final_pos.getX()+init_pos.getX(), final_pos.getY()+init_pos.getY(), final_pos.getRotation());
        TrajectoryGenerator.generateTrajectory(
            init_pos,
            inter_points,
            final_pos,
            new TrajectoryConfig(
                robotDrive.getAutoConstants().kMaxSpeedMetersPerSecond,
                robotDrive.getAutoConstants().kMaxAccelerationMetersPerSecondSquared
            )
                .setKinematics(robotDrive.getDriveConstants().kDriveKinematics)
                .addConstraint(
                    new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                            robotDrive.getDriveConstants().ksVolts, 
                            robotDrive.getDriveConstants().kvVoltSecondsPerMeter, 
                            robotDrive.getDriveConstants().kaVoltSecondsSquaredPerMeter
                        ), 
                        robotDrive.getDriveConstants().kDriveKinematics, 
                        maxVoltage
                    )
                )
        );
        long endTime = RobotController.getFPGATime();
        System.out.println("Trajectory generation time: " + (endTime - startTime) + "ms");
        LogKitten.v("Trajectory generation time: " + (endTime - startTime) + "ms");
    }
    
}
