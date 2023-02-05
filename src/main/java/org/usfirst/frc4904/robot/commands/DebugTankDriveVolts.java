package org.usfirst.frc4904.robot.commands;

import java.io.Console;

import org.usfirst.frc4904.standard.commands.motor.MotorSet;
import org.usfirst.frc4904.standard.custom.sensors.InvalidSensorException;
import org.usfirst.frc4904.standard.subsystems.chassis.Chassis;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class DebugTankDriveVolts extends ParallelCommandGroup {
    private final Chassis driveBase;
    private final double left;
    private final double right;
    protected final MotorSet[] motorSpins;
	  protected double[] motorSpeeds;
	  protected final Motor[] motors;

    public DebugTankDriveVolts(Chassis chassis, double left, double right) {
        super();
        this.driveBase = chassis;
        this.left = left;
        this.right = right;
        motors = chassis.getMotors();        
        addRequirements(chassis);
		    motorSpins = new MotorSet[motors.length];
		    for (int i = 0; i < motors.length; i++) {
			    motorSpins[i] = new MotorSet(motors[i].getName(), motors[i]);
	    	}
		    addCommands(motorSpins);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) throws InvalidSensorException {
        Motor[] motors = driveBase.getMotors();
        if (motors.length == 2) {
          motorSpins[0].setVoltage(leftVolts);
          motorSpins[1].setVoltage(rightVolts);
          System.out.println("We have 4 motors wtf are you doing");
        } else if (motors.length == 4) {
          motorSpins[0].setVoltage(leftVolts);
          motorSpins[1].setVoltage(leftVolts);
          motorSpins[2].setVoltage(rightVolts);
          motorSpins[3].setVoltage(rightVolts);
          System.out.println("voltage should be set");
          System.out.println(motorSpins[3].getVoltage());
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
