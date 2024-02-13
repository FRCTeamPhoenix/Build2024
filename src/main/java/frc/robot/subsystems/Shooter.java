// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.Constants.ModuleConstants;

public class Shooter {
  private final CANSparkMax m_leftMotor;
  private final SparkPIDController m_PIDController;
  private final RelativeEncoder m_encoder;

  private final CANSparkMax m_rightMotor;

  // TODO: Confirm that constants used apply to the shooter. We may need new constants specific to the shooter.
  public Shooter(int leftCANID, int rightCANID) {
    m_leftMotor = new CANSparkMax(leftCANID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightCANID, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the left and right SPARKS MAX.
    m_encoder = m_leftMotor.getEncoder();
    m_PIDController = m_leftMotor.getPIDController();
    m_PIDController.setFeedbackDevice(m_encoder);

    // Apply position and velocity conversion factors for the motor encoders. The
    // native unit for velocity is RPM, but we want meters per second for human input.
    m_encoder.setVelocityConversionFactor(ModuleConstants.kShooterEncoderVelocityFactor);

    // Set the PID gains for the motors. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_PIDController.setP(ModuleConstants.kShooterP);
    m_PIDController.setI(ModuleConstants.kShooterI);
    m_PIDController.setD(ModuleConstants.kShooterD);
    m_PIDController.setFF(ModuleConstants.kShooterFF);
    m_PIDController.setOutputRange(-1, 1);

    m_leftMotor.setIdleMode(ModuleConstants.kShooterMotorIdleMode);
    m_leftMotor.setSmartCurrentLimit(40);

    m_rightMotor.setIdleMode(ModuleConstants.kShooterMotorIdleMode);
    m_rightMotor.setSmartCurrentLimit(40);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_leftMotor.burnFlash();
    m_rightMotor.burnFlash();

    m_rightMotor.follow(m_leftMotor, true);
  }

  public void setDesiredVelocity(double desiredVelocity) {
    // Command intake motors towards their respective setpoints, with one motor being flipped
    // TODO: Note: CAD believes that both motors will run counter-clockwise. We may need to change these values later.
    m_PIDController.setReference(-desiredVelocity, ControlType.kVelocity);
  }
}