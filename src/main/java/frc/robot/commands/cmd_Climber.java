// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
        climber.setPower(voltage);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
