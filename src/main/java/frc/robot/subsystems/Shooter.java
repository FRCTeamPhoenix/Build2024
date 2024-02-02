// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ModuleConstants;

public class Shooter {
  private final CANSparkMax m_shooterMotorLeft;
  private final CANSparkMax m_shooterMotorRight;

  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  private final SparkPIDController m_shooterMotorRightPIDController;
  private final SparkPIDController m_shooterMotorLeftPIDController;

  // TODO: Confirm that constants used apply to the shooter. We may need new constants specific to the shooter.
  public Shooter(int leftCANID, int rightCANID) {
    m_shooterMotorRight = new CANSparkMax(rightCANID, MotorType.kBrushless);
    m_shooterMotorLeft = new CANSparkMax(leftCANID, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_shooterMotorLeft.restoreFactoryDefaults();
    m_shooterMotorRight.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the left and right SPARKS MAX.
    m_leftEncoder = m_shooterMotorLeft.getEncoder();
    m_rightEncoder = m_shooterMotorRight.getEncoder();
    m_shooterMotorLeftPIDController = m_shooterMotorLeft.getPIDController();
    m_shooterMotorLeftPIDController.setFeedbackDevice(m_leftEncoder);
    m_shooterMotorRightPIDController = m_shooterMotorRight.getPIDController();
    m_shooterMotorRightPIDController.setFeedbackDevice(m_rightEncoder);

    // Apply position and velocity conversion factors for the motor encoders. The
    // native unit for velocity is RPM, but we want meters per second for human input.
    m_leftEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    m_rightEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Set the PID gains for the motors. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_shooterMotorLeftPIDController.setP(ModuleConstants.kIntakeP);
    m_shooterMotorLeftPIDController.setI(ModuleConstants.kIntakeI);
    m_shooterMotorLeftPIDController.setD(ModuleConstants.kIntakeD);
    m_shooterMotorLeftPIDController.setFF(ModuleConstants.kIntakeFF);
    m_shooterMotorLeftPIDController.setOutputRange(ModuleConstants.kIntakeMinOutput,
        ModuleConstants.kIntakeMaxOutput);

    m_shooterMotorRightPIDController.setP(ModuleConstants.kIntakeP);
    m_shooterMotorRightPIDController.setI(ModuleConstants.kIntakeI);
    m_shooterMotorRightPIDController.setD(ModuleConstants.kIntakeD);
    m_shooterMotorRightPIDController.setFF(ModuleConstants.kIntakeFF);
    m_shooterMotorRightPIDController.setOutputRange(ModuleConstants.kIntakeMinOutput,
        ModuleConstants.kIntakeMaxOutput);

    m_shooterMotorLeft.setIdleMode(ModuleConstants.kIntakeMotorIdleMode);
    m_shooterMotorLeft.setSmartCurrentLimit(ModuleConstants.kIntakeMotorCurrentLimit);
    m_shooterMotorRight.setIdleMode(ModuleConstants.kIntakeMotorIdleMode);
    m_shooterMotorRight.setSmartCurrentLimit(ModuleConstants.kIntakeMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_shooterMotorLeft.burnFlash();
    m_shooterMotorRight.burnFlash();
  }

  public void setDesiredVelocity(double desiredVelocity) {
    // Command intake motors towards their respective setpoints, with one motor being flipped
    // TODO: Note: CAD believes that both motors will run counter-clockwise. We may need to change these values later.
    m_shooterMotorLeftPIDController.setReference(-desiredVelocity, CANSparkMax.ControlType.kVelocity);
    m_shooterMotorRightPIDController.setReference(-desiredVelocity, CANSparkMax.ControlType.kVelocity);
  }
}