// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

public class cmd_StopArm extends Command {
  private final Arm m_arm;
  //private final double m_setPoint;

  private double currentPositionToHold=0.0;

  public cmd_StopArm(Arm arm) {
    m_arm = arm;
    addRequirements(m_arm);

  }

  @Override
  public void initialize() {
    currentPositionToHold = m_arm.getArmPosition();   
    SmartDashboard.putNumber("Hold Angle",currentPositionToHold);
  }

  @Override
  public void execute() {
    m_arm.holdPosition(currentPositionToHold);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
