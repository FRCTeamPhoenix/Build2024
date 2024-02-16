// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.General;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

public class cmd_MoveArmDown extends Command {
  private final Arm m_arm;
  //private final double m_setPoint;


  public cmd_MoveArmDown(Arm arm) {
    m_arm = arm;
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    m_arm.moveArmDown();
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
