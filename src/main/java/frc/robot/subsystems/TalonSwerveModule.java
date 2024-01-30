// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class TalonSwerveModule {
  private final TalonFX m_drivingTalon;
  private final CurrentLimitsConfigs m_currentLimit;
  private final CANSparkMax m_turningSparkMax;
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);

  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with TalonFX driving motors, SPARK MAX turning motors, and a Through Bore
   * Encoder.
   */
  public TalonSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingTalon = new TalonFX(drivingCANId);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    m_currentLimit = new CurrentLimitsConfigs();
    m_currentLimit.SupplyCurrentLimit = ModuleConstants.kDrivingMotorCurrentLimit;
  
    // Factory reset, so we get the SPARK MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_turningSparkMax.restoreFactoryDefaults();

    // Configuring TalonFX
    TalonFXConfiguration configs = new TalonFXConfiguration();

    // Voltage-based velocity requires a feed forward to account for the back-emf of the motor
    configs.Slot0.kP = ModuleConstants.kTalonDrivingP;
    configs.Slot0.kI = ModuleConstants.kTalonDrivingI;
    configs.Slot0.kD = ModuleConstants.kTalonDrivingD; 
    configs.Slot0.kV = ModuleConstants.kTalonDrivingV;
    configs.Voltage.PeakForwardVoltage = ModuleConstants.kTalonDrivingPeakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = ModuleConstants.kTalonDrivingPeakReverseVoltage;

    // Retry config apply up to 5 times, report if failure
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_drivingTalon.getConfigurator().apply(configs);
      m_drivingTalon.getConfigurator().apply(m_currentLimit);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    // Setup encoders and PID controllers for the turning SPARK MAX.
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_drivingTalon.setNeutralMode(ModuleConstants.kDrivingMotorNeutralMode);
    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);

    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingTalon.getConfigurator().setPosition(0.0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingTalon.getVelocity().getValueAsDouble() * Constants.ModuleConstants.kTalonEncoderFactor,
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }
  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
            m_drivingTalon.getPosition().getValueAsDouble() * Constants.ModuleConstants.kTalonEncoderFactor,
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning motors towards their respective setpoints.
    m_drivingTalon.setControl(m_voltageVelocity.withVelocity(optimizedDesiredState.speedMetersPerSecond / Constants.ModuleConstants.kTalonEncoderFactor));
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
     m_drivingTalon.getConfigurator().setPosition(0.0);
  }
}
