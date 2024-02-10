/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * Before Running:
 * Open shuffleBoard, select File->Load Layout and select the 
 * shuffleboard.json that is in the root directory of this example
 */

/**
 * REV Smart Motion Guide
 * 
 * The SPARK MAX includes a new control mode, REV Smart Motion which is used to 
 * control the position of the motor, and includes a max velocity and max 
 * acceleration parameter to ensure the motor moves in a smooth and predictable 
 * way. This is done by generating a motion profile on the fly in SPARK MAX and 
 * controlling the velocity of the motor to follow this profile.
 * 
 * Since REV Smart Motion uses the velocity to track a profile, there are only 
 * two steps required to configure this mode:
 *    1) Tune a velocity PID loop for the mechanism
 *    2) Configure the smart motion parameters
 * 
 * Tuning the Velocity PID Loop
 * 
 * The most important part of tuning any closed loop control such as the velocity 
 * PID, is to graph the inputs and outputs to understand exactly what is happening. 
 * For tuning the Velocity PID loop, at a minimum we recommend graphing:
 *
 *    1) The velocity of the mechanism (‘Process variable’)
 *    2) The commanded velocity value (‘Setpoint’)
 *    3) The applied output
 *
 * This example will use ShuffleBoard to graph the above parameters. Make sure to
 * load the shuffleboard.json file in the root of this directory to get the full
 * effect of the GUI layout.
 */
public class Arm extends SubsystemBase {
  private CANSparkMax m_leftMotor, m_rightMotor;
  private SparkPIDController m_leftPidController, m_rightPidController;
  private RelativeEncoder m_leftEncoder, m_rightEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  public Arm(int rightDeviceID, int leftDeviceID) {
    // initialize motor
    m_leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushless);

    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    m_leftPidController = m_leftMotor.getPIDController();
    m_leftEncoder = m_leftMotor.getEncoder();

    m_rightPidController = m_leftMotor.getPIDController();
    m_rightEncoder = m_leftMotor.getEncoder();

    // PID coefficients
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    m_leftPidController.setP(kP);
    m_leftPidController.setI(kI);
    m_leftPidController.setD(kD);
    m_leftPidController.setIZone(kIz);
    m_leftPidController.setFF(kFF);
    m_leftPidController.setOutputRange(kMinOutput, kMaxOutput);

    m_rightPidController.setP(kP);
    m_rightPidController.setI(kI);
    m_rightPidController.setD(kD);
    m_rightPidController.setIZone(kIz);
    m_rightPidController.setFF(kFF);
    m_rightPidController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    m_leftPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_leftPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_leftPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_leftPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    m_rightPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_rightPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_rightPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_rightPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
  }

  @Override
  public void periodic() {
      // This method will be called once per scheduler run
      
  }

  public void moveArm(double setpoint) {
    m_leftPidController.setReference(-setpoint, CANSparkMax.ControlType.kSmartMotion);
    m_rightPidController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    
    SmartDashboard.putNumber("Output", m_rightMotor.getAppliedOutput());
  }

  public void killArm(){
    m_leftPidController.setReference(0.0, ControlType.kVoltage);
    m_rightPidController.setReference(0.0, ControlType.kVoltage);
  }
  
  public double currentAngle(){
    return m_leftEncoder.getPosition();
  }
}