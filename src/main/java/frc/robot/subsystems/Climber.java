// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final CANSparkMax m_sparkMaxLeft;
    private final CANSparkMax m_sparkMaxRight;

    public Climber(int canIDLeft, int canIDRight) {
        m_sparkMaxLeft = new CANSparkMax(canIDLeft, MotorType.kBrushless);
        m_sparkMaxRight = new CANSparkMax(canIDRight, MotorType.kBrushless);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_sparkMaxLeft.restoreFactoryDefaults();
        m_sparkMaxRight.restoreFactoryDefaults();


        m_sparkMaxLeft.setIdleMode(ClimberConstants.kClimberIdleMode);
        m_sparkMaxRight.setIdleMode(ClimberConstants.kClimberIdleMode);

        m_sparkMaxLeft.setSmartCurrentLimit(20);
        m_sparkMaxRight.setSmartCurrentLimit(20);

        m_sparkMaxRight.follow(m_sparkMaxLeft);
        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_sparkMaxLeft.burnFlash();
        m_sparkMaxRight.burnFlash();
    }

    public void setPower(double voltage) {
        m_sparkMaxLeft.setVoltage(voltage);
    }
}