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

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.LogKitten;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RamseteCommandDebug extends RamseteCommand{
    double elapsed = 0; 
    ScheduledExecutorService logger = Executors.newScheduledThreadPool(1);
    ArrayList<ArrayList> actualdata = new ArrayList<ArrayList>();

    public RamseteCommandDebug(Trajectory trajectory,
    Supplier<Pose2d> pose,
    RamseteController controller,
    SimpleMotorFeedforward feedforward,
    DifferentialDriveKinematics kinematics,
    Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
    PIDController leftController,
    PIDController rightController,
    BiConsumer<Double, Double> outputVolts,
    Subsystem... requirements) {
        super(trajectory, pose, controller, feedforward, kinematics, wheelSpeeds, leftController, rightController, outputVolts, requirements);
    }
    public void initialize() {
        super.initialize();
        logger.scheduleAtFixedRate(() -> {
            DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(RobotMap.Component.driveConst.kTrackwidthMeters);
            var wheel_speeds = RobotMap.Component.SplinesDrive.getWheelSpeeds();
            var chassis_speeds = kinematics.toChassisSpeeds(wheel_speeds);
            var actual_velocity = chassis_speeds.vxMetersPerSecond;
            var actual_curvature = chassis_speeds.omegaRadiansPerSecond/actual_velocity;
            var actual_x = RobotMap.Component.SplinesDrive.getPose().getX();
            var actual_y = RobotMap.Component.SplinesDrive.getPose().getY();
            actualdata.add(new ArrayList<Double>(Arrays.asList(elapsed, actual_velocity, actual_curvature, actual_x, actual_y)));
            elapsed += 0.02;
        }, 0, 20, TimeUnit.MILLISECONDS);
    }
    public void execute() {
        super.execute();
    }
    public void end(boolean interrupted) {
        super.end(interrupted);
        try {
            FileWriter writer = new FileWriter("/home/lvuser/actualdata.csv");
            for (ArrayList<Double> row : actualdata) {
              writer.write(String.join(",", row.stream().map(Object::toString).collect(Collectors.toList())));
              writer.write("\n");
            }
            writer.close();
          } catch (IOException e) {
            LogKitten.ex(e);
        }
        logger.shutdownNow();
    }
    public boolean isFinished() {
        return super.isFinished();
    }   
}
