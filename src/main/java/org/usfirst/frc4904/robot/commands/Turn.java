package org.usfirst.frc4904.robot.commands;

import org.usfirst.frc4904.standard.commands.motor.MotorConstant;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;

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