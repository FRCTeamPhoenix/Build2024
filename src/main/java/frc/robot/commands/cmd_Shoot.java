// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.General;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class cmd_Shoot extends Command {
  private final Shooter m_shooter;

  public cmd_Shoot(Shooter shooter) {
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    m_shooter.setDesiredVelocity(16.5);
  }

  @Override
  public void end(boolean interrupted) {
    if (General.LOGGING)
      System.out.println("End Move Arm Down");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
