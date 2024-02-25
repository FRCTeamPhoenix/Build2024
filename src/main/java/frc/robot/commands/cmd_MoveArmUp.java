// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.General;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

public class cmd_MoveArmUp extends Command {
    private final Arm m_arm;
    //private final double m_setPoint;

    private double m_dIncrement;

    public cmd_MoveArmUp(Arm arm, double dIncrement) {
        m_arm = arm;
        m_dIncrement = dIncrement;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_arm.moveArmUp(m_dIncrement);
    }

    @Override
    public void end(boolean interrupted) {
        if (General.LOGGING) {
            System.out.println("End of Move Arm UP");
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
