// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ClimberConstants;

public class Climber {
    private final CANSparkMax m_sparkMax;


    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public Climber(int canID) {
        m_sparkMax = new CANSparkMax(canID, MotorType.kBrushed);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_sparkMax.restoreFactoryDefaults();


        m_sparkMax.setIdleMode(ClimberConstants.kClimberIdleMode);

        m_sparkMax.setSmartCurrentLimit(40);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_sparkMax.burnFlash();
    }

    public void setPower(double voltage) {
        m_sparkMax.setVoltage(voltage);
    }
}