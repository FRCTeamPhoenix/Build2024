// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class cmd_DriveAndIntake extends Command {
  private DriveSubsystem m_robotDrive;
  private Intake m_intake;

  public cmd_DriveAndIntake(DriveSubsystem robotDrive, Intake intake) {
    m_robotDrive = robotDrive;
    m_intake = intake;

    addRequirements(m_robotDrive);
  }

  @Override
  public void initialize() {
    m_intake.setDesiredVelocity(2.5);
  }

  @Override
  public void execute() {
    if (SmartDashboard.getString("FRCNote", "Not Found").equals("Found")){
      m_robotDrive.drive(0.0, 0.0, 0.0, false, false);
      m_intake.stopIntake();
    }
    else {
      m_robotDrive.drive(3.0, 0.0, 0.0, false, false);
    }
  }

  @Override
  public void end(boolean interrupted) {      
    m_robotDrive.drive(0.0, 0.0, 0.0, false, false);
    m_intake.stopIntake();
  }

  @Override
  public boolean isFinished() {
    // if the robot is alligned with the note then finish the command
    if (SmartDashboard.getString("FRCNote", "Not Found").equals("Found")){
      return true;
    }
    return false;
  }
}
