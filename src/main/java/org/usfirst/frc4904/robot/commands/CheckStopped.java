package org.usfirst.frc4904.robot.commands;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.Util;
import org.usfirst.frc4904.standard.custom.CustomPIDSourceType;
import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CheckStopped extends CommandBase {
    public CheckStopped(CANTalonEncoder talon) {
        int stoppedFwd = talon.getFwdLimitSwitchClosed();
        int stoppedRev = talon.getRevLimitSwitchClosed();
        LogKitten.wtf("Forward: " + String.valueOf(stoppedFwd));
        LogKitten.wtf("Reverse: " + String.valueOf(stoppedRev));
    }


}
