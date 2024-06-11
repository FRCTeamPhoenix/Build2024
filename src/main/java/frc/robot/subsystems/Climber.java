// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    public final CANSparkMax m_sparkMaxLeft;
    public final CANSparkMax m_sparkMaxRight;
    private final SparkPIDController pidController;
    private final RelativeEncoder relativeEncoder;

    public Climber(int canIDLeft, int canIDRight) {
        m_sparkMaxLeft = new CANSparkMax(canIDLeft, MotorType.kBrushless);
        m_sparkMaxRight = new CANSparkMax(canIDRight, MotorType.kBrushless);

        relativeEncoder = m_sparkMaxLeft.getEncoder();
        pidController = m_sparkMaxLeft.getPIDController();
        pidController.setFeedbackDevice(relativeEncoder);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_sparkMaxLeft.restoreFactoryDefaults();
        m_sparkMaxRight.restoreFactoryDefaults();

        pidController.setP(0.04);
        pidController.setFF(0.001);

        m_sparkMaxLeft.setIdleMode(ClimberConstants.kClimberIdleMode);
        m_sparkMaxRight.setIdleMode(ClimberConstants.kClimberIdleMode);

        m_sparkMaxLeft.setSmartCurrentLimit(40);
        m_sparkMaxRight.setSmartCurrentLimit(40);

        //m_sparkMaxRight.setInverted(true);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_sparkMaxLeft.burnFlash();
        m_sparkMaxRight.burnFlash();
    }

    public void setPower(double voltage) {
        SmartDashboard.putNumber("climber volt", voltage);
        if (encoderPosition() <= -68.7 && voltage < 0){
            m_sparkMaxLeft.setVoltage(0.0);
            }
        else if ((encoderPosition() <= -68.7 && voltage >= 0) || encoderPosition() > -68.7){
            m_sparkMaxLeft.setVoltage(voltage);
        }
    }
    public void setPosition(double position) {
        pidController.setReference(position, ControlType.kPosition);
    }
    public double encoderPosition(){
        return relativeEncoder.getPosition();
    }
}