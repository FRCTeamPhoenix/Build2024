// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* This is derived from MAXSwerve module code by retaining only the turning motor code 
 * and doing some renaming - also added a local ArmConstants class that should be 
 * moved to constants.java.  For now, because the swerve code was all in radians this 
 * is also in radians.  There are also edits to the arm control code in robot.java to 
 * make the units consistent
*/

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

//import frc.robot.Constants.ModuleConstants;


public class Arm extends SubsystemBase {
  
  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;

  private final AbsoluteEncoder m_armEncoder;

  private final SparkPIDController m_armPIDController;

  private double armPositionRequest;

  //private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public Arm(int leftCANID, int rightCANID) {
    
    // this shoould become part of constants.java
    final class ArmConstants {
      public static final double kArmP = 1.0;
      public static final double kArmI = 0;
      public static final double kArmD = 0;
      public static final double kArmFF = 0;
      public static final double kArmMinOutput = -0.7;
      public static final double kArmMaxOutput = 0.7;
      public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;
      public static final int kArmMotorCurrentLimit = 20; // amps
  
      public static final double kArmEncoderPositionFactor = (2 * Math.PI); // radians
      public static final double kArmEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
      public static final double kArmEncoderPositionPIDMinInput = 0; // radians
      public static final double kArmEncoderPositionPIDMaxInput = kArmEncoderPositionFactor; // radians
  }
  

    m_leftMotor = new CANSparkMax(leftCANID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightCANID, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
   
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.

    m_armEncoder = m_leftMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_armEncoder.setPositionConversionFactor(ArmConstants.kArmEncoderPositionFactor);
   
    m_armPIDController = m_leftMotor.getPIDController();

    m_armPIDController.setFeedbackDevice(m_armEncoder);

    m_leftMotor.setInverted(true);


    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_armEncoder.setPositionConversionFactor(ArmConstants.kArmEncoderPositionFactor);
    m_armEncoder.setVelocityConversionFactor(ArmConstants.kArmEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    //m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.

    /* this code was changed back to enable wrpping and configure the sensor range */
    m_armPIDController.setPositionPIDWrappingEnabled(true);
    m_armPIDController.setPositionPIDWrappingMinInput(ArmConstants.kArmEncoderPositionPIDMinInput);
    m_armPIDController.setPositionPIDWrappingMaxInput(ArmConstants.kArmEncoderPositionPIDMaxInput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_armPIDController.setP(ArmConstants.kArmP);
    m_armPIDController.setI(ArmConstants.kArmI);
    m_armPIDController.setD(ArmConstants.kArmD);
    m_armPIDController.setFF(ArmConstants.kArmFF);
    m_armPIDController.setOutputRange(ArmConstants.kArmMinOutput,
        ArmConstants.kArmMaxOutput);

    m_leftMotor.setIdleMode(ArmConstants.kArmMotorIdleMode);
    m_leftMotor.setSmartCurrentLimit(ArmConstants.kArmMotorCurrentLimit);

    m_rightMotor.setIdleMode(ArmConstants.kArmMotorIdleMode);
    m_rightMotor.setSmartCurrentLimit(ArmConstants.kArmMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_leftMotor.burnFlash();

    m_rightMotor.follow(m_leftMotor, true);
    m_rightMotor.burnFlash();


    armPositionRequest = getArmPosition();  // initialize to where we are

  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public double getArmPosition() {
    return m_armEncoder.getPosition();
  }

  public void moveArmUp() {
    double arm_range = 2*Math.PI;  //encoder native units appear to 0-1.0 = 360 degrees
        /* this code does not belong in robot.java  for now it is hacked in to provide a way to test the 
     * arm code -- note that this ramps the setpoint  -- real code for a ramp button command should probably
     * ramp from get_position() For now arm position is in radians.
     */
    double angle = this.getArmPositionRequest();
    angle += arm_range/20;
    if (angle > arm_range/2) {
      angle = Math.PI;  //180 degrees
    }
    else if (angle < arm_range/50) {
      angle = arm_range/50;
    }
    this.setArmPosition(angle);
  }

  public void moveArmDown() {
    double arm_range = 2*Math.PI;  //encoder native units appear to 0-1.0 = 360 degrees
        /* this code does not belong in robot.java  for now it is hacked in to provide a way to test the 
     * arm code -- note that this ramps the setpoint  -- real code for a ramp button command should probably
     * ramp from get_position() For now arm position is in radians.
     */
    double angle = this.getArmPositionRequest();
    angle -= arm_range/20;
    if (angle > arm_range/2) {
      angle = Math.PI;  //180 degrees
    }
    else if (angle < arm_range/50) {
      angle = arm_range/50;
    }
    this.setArmPosition(angle);
  }

  public void holdPosition() {
    setArmPosition(getArmPosition());
  }

  public double getArmPositionRequest() {
    return armPositionRequest;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setArmPosition(double angle) {
    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_armPIDController.setReference(angle, ControlType.kPosition);
    armPositionRequest = angle;
  }

  public void killArm() {
    m_armPIDController.setReference(0.0, ControlType.kVoltage);
  }
}
