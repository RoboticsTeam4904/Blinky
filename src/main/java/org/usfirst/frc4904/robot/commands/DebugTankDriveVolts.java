package org.usfirst.frc4904.robot.commands;

import org.usfirst.frc4904.standard.custom.sensors.InvalidSensorException;
import org.usfirst.frc4904.standard.subsystems.chassis.Chassis;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DebugTankDriveVolts extends CommandBase{
    private final Chassis driveBase;
    private final double left;
    private final double right;

    public DebugTankDriveVolts(Chassis chassis, double left, double right) {
        super();
        this.driveBase = chassis;
        this.left = left;
        this.right = right;
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) throws InvalidSensorException {
        Motor[] motors = driveBase.getMotors();
        if (motors.length == 2) {
          driveBase.getMotors()[0].setVoltage(leftVolts);
          driveBase.getMotors()[1].setVoltage(rightVolts);
        } else if (motors.length == 4) {
          driveBase.getMotors()[0].setVoltage(leftVolts);
          driveBase.getMotors()[1].setVoltage(leftVolts);
          driveBase.getMotors()[2].setVoltage(rightVolts);
          driveBase.getMotors()[3].setVoltage(rightVolts);
        } else {
          throw new InvalidSensorException("Invalid number of motors in drivebase");
        }
      }

    @Override
    public void initialize() {
        try {
            tankDriveVolts(left, right);
        } catch (InvalidSensorException e) {
            e.printStackTrace();
        }
    }
}
