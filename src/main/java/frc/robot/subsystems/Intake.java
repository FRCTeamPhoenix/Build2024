// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.Constants.IntakeConstants;;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_intakeMotor;
    private final RelativeEncoder m_encoder;
    private final SparkPIDController m_PIDController;

    // TODO: Confirm that constants used apply to the intake. We may need new constants specific to the intake.
    public Intake(int CANID) {
        m_intakeMotor = new CANSparkMax(CANID, MotorType.kBrushless);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_intakeMotor.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the left and right SPARKS MAX.
        m_encoder = m_intakeMotor.getEncoder();
        m_PIDController = m_intakeMotor.getPIDController();
        m_PIDController.setFeedbackDevice(m_encoder);

        // Apply position and velocity conversion factors for the motor encoders. The
        // native unit for velocity is RPM, but we want meters per second for human input.
        m_encoder.setVelocityConversionFactor(IntakeConstants.kIntakeEncoderVelocityFactor);

        // Set the PID gains for the motors. Note these are example gains, and you
        // may need to tune them for your own robot!
        m_PIDController.setP(IntakeConstants.kIntakeP);
        m_PIDController.setI(IntakeConstants.kIntakeI);
        m_PIDController.setD(IntakeConstants.kIntakeD);
        m_PIDController.setFF(IntakeConstants.kIntakeFF);
        m_PIDController.setOutputRange(-1, 1);

        m_intakeMotor.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        m_intakeMotor.setSmartCurrentLimit(40);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_intakeMotor.burnFlash();
    }

    public void setDesiredVelocity(double desiredVelocity) {
        // Command intake motors towards their respective setpoints, with one motor being flipped
        m_PIDController.setReference(desiredVelocity, ControlType.kVelocity);
    }

    public void stopIntake() {
        // Command intake motors towards their respective setpoints, with one motor being flipped
        m_PIDController.setReference(0.0, ControlType.kVoltage);
    }

    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    public void intakeNote(boolean noteInShooter) {
        if (!noteInShooter) {
            setDesiredVelocity(5.0);
        } else {
            setDesiredVelocity(0.0);
        }
    }

    public void loadNote(boolean noteInShooter) {
        if (noteInShooter) {
            setDesiredVelocity(2.5);
        } else {
            setDesiredVelocity(0.0);
        }
    }

    public void spitNote(boolean noteInShooter) {
        if (noteInShooter) {
            setDesiredVelocity(-0.5);
        } else {
            setDesiredVelocity(0.0);
        }
    }
}