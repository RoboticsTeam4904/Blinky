package org.usfirst.frc4904.standard.commands.motor;

import org.usfirst.frc4904.standard.subsystems.motor.Motor;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Constantly turns motor at specified speed.
 */

public class Turn extends MotorConstant {
    public Turn(String name, Motor motor, double motorSpeed) {
        super(name, motor, motorSpeed);
    }

    public Turn(Motor motor, double motorSpeed) {
        super(motor, motorSpeed);
    }
}