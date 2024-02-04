// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;

public class NEOMotorModule {
  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;

  private final double m_motorVelocityFactor;

  private final SparkPIDController m_PIDController;

  // TODO: Confirm that constants used apply to the intake. We may need new constants specific to the intake.
  public NEOMotorModule(int CANID, double velocityFactor, double P, double I, double D, double FF, IdleMode idleMode) {
    m_motor = new CANSparkMax(CANID, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_motor.restoreFactoryDefaults();

    m_motorVelocityFactor = velocityFactor;

    // Setup encoders and PID controllers for the left and right SPARKS MAX.
    m_encoder = m_motor.getEncoder();
    m_PIDController = m_motor.getPIDController();
    m_PIDController.setFeedbackDevice(m_encoder);

    // Apply position and velocity conversion factors for the motor encoders. The
    // native unit for velocity is RPM, but we want meters per second for human input.
    m_encoder.setVelocityConversionFactor(m_motorVelocityFactor);

    // Set the PID gains for the motors. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_PIDController.setP(P);
    m_PIDController.setI(I);
    m_PIDController.setD(D);
    m_PIDController.setFF(FF);
    m_PIDController.setOutputRange(-1, 1);

    m_motor.setIdleMode(idleMode);
    m_motor.setSmartCurrentLimit(40);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_motor.burnFlash();
  }

  public void setDesiredVelocity(double desiredVelocity) {
    // Command intake motors towards their respective setpoints, with one motor being flipped
    m_PIDController.setReference(desiredVelocity, CANSparkMax.ControlType.kVelocity);
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }
}