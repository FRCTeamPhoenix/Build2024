// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ModuleConstants;

public class Shooter {
  private final NEOMotorModule m_shooterMotorLeft;
  private final NEOMotorModule m_shooterMotorRight;

  // TODO: Confirm that constants used apply to the shooter. We may need new constants specific to the shooter.
  public Shooter(int leftCANID, int rightCANID) {
    m_shooterMotorLeft = new NEOMotorModule(
      leftCANID, 
      ModuleConstants.kShooterEncoderVelocityFactor, 
      ModuleConstants.kShooterP, 
      ModuleConstants.kShooterI, 
      ModuleConstants.kShooterD, 
      ModuleConstants.kShooterFF, 
      ModuleConstants.kShooterMotorIdleMode);

    m_shooterMotorRight = new NEOMotorModule(
      rightCANID, 
      ModuleConstants.kShooterEncoderVelocityFactor, 
      ModuleConstants.kShooterP, 
      ModuleConstants.kShooterI, 
      ModuleConstants.kShooterD, 
      ModuleConstants.kShooterFF, 
      ModuleConstants.kShooterMotorIdleMode);
  }

  public void setDesiredVelocity(double desiredVelocity) {
    // Command intake motors towards their respective setpoints, with one motor being flipped
    // TODO: Note: CAD believes that both motors will run counter-clockwise. We may need to change these values later.
    m_shooterMotorLeft.setDesiredVelocity(-desiredVelocity);
    m_shooterMotorRight.setDesiredVelocity(-desiredVelocity);
  }
}