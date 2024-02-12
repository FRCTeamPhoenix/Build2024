// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.ModuleConstants;

public class Intake {
  private final NEOMotorModule m_intakeMotor;

  // TODO: Confirm that constants used apply to the intake. We may need new constants specific to the intake.
  public Intake(int CANID) {
    m_intakeMotor = new NEOMotorModule(
      CANID, 
      ModuleConstants.kDrivingEncoderVelocityFactor, 
      ModuleConstants.kIntakeP, 
      ModuleConstants.kIntakeI, 
      ModuleConstants.kIntakeD, 
      ModuleConstants.kIntakeFF,
      ModuleConstants.kIntakeMotorIdleMode);
  }

  public void setDesiredVelocity(double desiredVelocity) {
    // Command intake motors towards their respective setpoints, with one motor being flipped
    m_intakeMotor.setDesiredVelocity(desiredVelocity);
  }

  public double getVelocity() {
    return m_intakeMotor.getVelocity();
  }

  public void intakeNote(boolean noteInShooter) {
    if (!noteInShooter){
      setDesiredVelocity(10.0);
    }
    else {
      setDesiredVelocity(0.0);
    }
  }

  public void loadNote(boolean noteInShooter) {
    if (noteInShooter){
      setDesiredVelocity(2.5);
    }
    else {
      setDesiredVelocity(0.0);
    }
  }
}