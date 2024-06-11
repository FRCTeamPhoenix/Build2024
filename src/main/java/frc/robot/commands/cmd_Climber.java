// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;


public class cmd_Climber extends Command {
    Climber climber;
    double voltage;

    /**
     * @param climber If you can't figure out what this means, please socially distance yourself from any code for the robot, and let software handle it.
     * @param voltage Max 12V. Positive moves the climbers down (moving the robot up).
     */
    public cmd_Climber(Climber climber, double voltage) {
        this.climber = climber;
        this.voltage = voltage;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //climber.setPower(voltage);
        SmartDashboard.putNumber("climber volt", voltage);
        if (climber.encoderPosition() <= -68.7 && voltage < 0){
            climber.m_sparkMaxLeft.setVoltage(0.0);
            climber.m_sparkMaxRight.setVoltage(0.0);
            }
        else if ((climber.encoderPosition() <= -68.7 && voltage >= 0) || climber.encoderPosition() > -68.7){
            climber.m_sparkMaxLeft.setVoltage(-voltage);
            climber.m_sparkMaxRight.setVoltage(voltage);
        } 
        // if (voltage > 0) {
        //     climber.setPosition(2.0);
        // }
        // else {
        //     climber.setPosition(-2.0);
        // }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
