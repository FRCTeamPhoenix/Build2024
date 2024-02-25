// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class cmd_IntakeNote extends Command {
    private final Intake m_intake;

    public cmd_IntakeNote(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        String status = SmartDashboard.getString("FRC-Note", "Not Found");
        if (status.equals("Captured") || status.equals("Found")) m_intake.setDesiredVelocity(0.0);
        else m_intake.setDesiredVelocity(1.5);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        String status = SmartDashboard.getString("FRC-Note", "Not Found");
        if (status.equals("Captured") || status.equals("Found")) return true;
        return false;
    }
}
