// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;

import frc.robot.Constants.ModuleConstants;

public class Arm {
  private final CANSparkMax m_armSparkMax;

  private final AbsoluteEncoder m_armEncoder;

  private final SparkPIDController m_armPIDController;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  // TODO: Confirm that constants used apply to the arm. We may need new constants specific to the arm.
  public Arm(int CANId) {
    m_armSparkMax = new CANSparkMax(CANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_armSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_armEncoder = m_armSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_armPIDController = m_armSparkMax.getPIDController();
    m_armPIDController.setFeedbackDevice(m_armEncoder);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_armEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    m_armEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_armEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_armPIDController.setPositionPIDWrappingEnabled(true);
    m_armPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_armPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_armPIDController.setP(ModuleConstants.kNeoDrivingP);
    m_armPIDController.setI(ModuleConstants.kNeoDrivingI);
    m_armPIDController.setD(ModuleConstants.kNeoDrivingD);
    m_armPIDController.setFF(ModuleConstants.kNeoDrivingFF);
    m_armPIDController.setOutputRange(ModuleConstants.kNeoDrivingMinOutput,
        ModuleConstants.kNeoDrivingMaxOutput);

    m_armSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_armSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_armSparkMax.burnFlash();
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public Rotation2d getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new Rotation2d(m_armEncoder.getPosition());
  }

  /**
   * Sets the desired angle for the module.
   *
   * @param desiredAngle Desired angle.
   */
  public void setDesiredAngle(double desiredAngle) {
    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_armPIDController.setReference(desiredAngle, CANSparkMax.ControlType.kPosition);
  }
}