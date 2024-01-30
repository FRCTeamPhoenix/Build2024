// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ModuleConstants;

public class Intake {
  private final CANSparkMax m_intakeMotor;

  private final RelativeEncoder m_intakeEncoder;

  private final SparkPIDController m_intakeMotorPIDController;

  public Intake(int leftCANID, int rightCANID) {
    m_intakeMotor = new CANSparkMax(leftCANID, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_intakeMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the left and right SPARKS MAX.
    m_intakeEncoder = m_intakeMotor.getEncoder();
    m_intakeMotorPIDController = m_intakeMotor.getPIDController();
    m_intakeMotorPIDController.setFeedbackDevice(m_intakeEncoder);

    // Apply position and velocity conversion factors for the motor encoders. The
    // native unit for velocity is RPM, but we want meters per second for human input.
    m_intakeEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Set the PID gains for the motors. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_intakeMotorPIDController.setP(ModuleConstants.kIntakeP);
    m_intakeMotorPIDController.setI(ModuleConstants.kIntakeI);
    m_intakeMotorPIDController.setD(ModuleConstants.kIntakeD);
    m_intakeMotorPIDController.setFF(ModuleConstants.kIntakeFF);
    m_intakeMotorPIDController.setOutputRange(ModuleConstants.kIntakeMinOutput,
        ModuleConstants.kIntakeMaxOutput);

    m_intakeMotor.setIdleMode(ModuleConstants.kIntakeMotorIdleMode);
    m_intakeMotor.setSmartCurrentLimit(ModuleConstants.kIntakeMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_intakeMotor.burnFlash();
  }

  public void setDesiredVelocity(double desiredVelocity) {
    // Command intake motors towards their respective setpoints, with one motor being flipped
    m_intakeMotorPIDController.setReference(desiredVelocity, CANSparkMax.ControlType.kVelocity);
  }
}