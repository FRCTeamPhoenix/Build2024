// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
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
    m_encoderLeft.setVelocityConversionFactor(ShooterConstants.kShooterEncoderVelocityFactor);
    m_encoderRight.setVelocityConversionFactor(ShooterConstants.kShooterEncoderVelocityFactor);

    // Set the PID gains for the motors. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_PIDControllerLeft.setP(ShooterConstants.kShooterP);
    m_PIDControllerLeft.setI(ShooterConstants.kShooterI);
    m_PIDControllerLeft.setD(ShooterConstants.kShooterD);
    m_PIDControllerLeft.setFF(ShooterConstants.kShooterFF);
    m_PIDControllerLeft.setOutputRange(-1, 1);

    m_PIDControllerRight.setP(ShooterConstants.kShooterP);
    m_PIDControllerRight.setI(ShooterConstants.kShooterI);
    m_PIDControllerRight.setD(ShooterConstants.kShooterD);
    m_PIDControllerRight.setFF(ShooterConstants.kShooterFF);
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
    m_PIDControllerLeft.setReference(desiredVelocity, ControlType.kVelocity);
    m_PIDControllerRight.setReference(desiredVelocity*SmartDashboard.getNumber("PercentSpin", 0.7), ControlType.kVelocity);
  }

  public void killShooter(){
    m_PIDControllerLeft.setReference(0.0, ControlType.kVoltage);
    m_PIDControllerRight.setReference(0.0, ControlType.kVoltage);
  }
  public double getVelocity(){
    return m_encoderLeft.getVelocity();
  }
}