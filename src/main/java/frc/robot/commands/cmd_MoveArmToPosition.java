// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class cmd_MoveArmToPosition extends Command {
    private final Arm m_arm;
    private final double m_setPoint;

    /**
     * Command the arm to move to a setpoint.
     *
     * @param setPoint Where to move the arm
     * @param speed    The speed the arm will move
     * @param arm      The Arm Subsystem
     */
    public cmd_MoveArmToPosition(double setPoint, double speed, Arm arm) {
        m_setPoint = setPoint;
        m_arm = arm;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_arm.setArmPosition(m_setPoint);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("IsAtPosition", m_arm.isAtPosition(m_setPoint));
        return m_arm.isAtPosition(m_setPoint);
    }
}
