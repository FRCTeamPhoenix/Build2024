// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.Constants.ShooterConstants;;

public class Shooter {
  private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor;

  private final SparkPIDController m_PIDControllerLeft;
  private final SparkPIDController m_PIDControllerRight;

  private final RelativeEncoder m_encoderLeft;
  private final RelativeEncoder m_encoderRight;

  // TODO: Confirm that constants used apply to the shooter. We may need new constants specific to the shooter.
  public Shooter(int leftCANID, int rightCANID) {
    m_leftMotor = new CANSparkMax(leftCANID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightCANID, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the left and right SPARKS MAX.
    m_encoderLeft = m_leftMotor.getEncoder();
    m_PIDControllerLeft = m_leftMotor.getPIDController();
    m_PIDControllerLeft.setFeedbackDevice(m_encoderLeft);

    m_encoderRight = m_rightMotor.getEncoder();
    m_PIDControllerRight = m_rightMotor.getPIDController();
    m_PIDControllerRight.setFeedbackDevice(m_encoderRight);

    m_leftMotor.setInverted(true);

    // Apply position and velocity conversion factors for the motor encoders. The
    // native unit for velocity is RPM, but we want meters per second for human input.
    m_encoderLeft.setVelocityConversionFactor(ModuleConstants.kShooterEncoderVelocityFactor);
    m_encoderRight.setVelocityConversionFactor(ModuleConstants.kShooterEncoderVelocityFactor);

    // Set the PID gains for the motors. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_PIDControllerLeft.setP(ModuleConstants.kShooterP);
    m_PIDControllerLeft.setI(ModuleConstants.kShooterI);
    m_PIDControllerLeft.setD(ModuleConstants.kShooterD);
    m_PIDControllerLeft.setFF(ModuleConstants.kShooterFF);
    m_PIDControllerLeft.setOutputRange(-1, 1);

    m_PIDControllerRight.setP(ModuleConstants.kShooterP);
    m_PIDControllerRight.setI(ModuleConstants.kShooterI);
    m_PIDControllerRight.setD(ModuleConstants.kShooterD);
    m_PIDControllerRight.setFF(ModuleConstants.kShooterFF);
    m_PIDControllerRight.setOutputRange(-1, 1);




    m_leftMotor.setIdleMode(ShooterConstants.kShooterMotorIdleMode);
    m_leftMotor.setSmartCurrentLimit(40);

    m_rightMotor.setIdleMode(ShooterConstants.kShooterMotorIdleMode);
    m_rightMotor.setSmartCurrentLimit(40);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_leftMotor.burnFlash();

    // m_rightMotor.follow(m_leftMotor, true);
    m_rightMotor.burnFlash();

  }

  public void setDesiredVelocity(double desiredVelocity) {
    // Command intake motors towards their respective setpoints, with one motor being flipped
    // TODO: Note: CAD believes that both motors will run counter-clockwise. We may need to change these values later.
    m_PIDControllerLeft.setReference(desiredVelocity * 0.6, ControlType.kVelocity);
    m_PIDControllerRight.setReference(desiredVelocity * 0.6, ControlType.kVelocity);
  }
}