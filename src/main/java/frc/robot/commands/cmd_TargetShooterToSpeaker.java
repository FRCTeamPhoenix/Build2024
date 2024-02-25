// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PhotonClass;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class cmd_TargetShooterToSpeaker extends Command {
    private final Arm m_arm;
    private final InterpolatingDoubleTreeMap m_interpolator;
    private double calculatedSetPoint;

    private double range = 0.0;

    private PhotonTrackedTarget result;

    /**
     * Command the arm to move to a setpoint.
     *
     * @param interpolator Interpolating Tree Map for shooter vaules
     * @param arm          The Arm Subsystem
     */
    public cmd_TargetShooterToSpeaker(InterpolatingDoubleTreeMap interpolator, PhotonClass photonCam, Arm arm, boolean isAllianceRed) {
        m_interpolator = interpolator;
        m_arm = arm;

        if (isAllianceRed) result = photonCam.getAprilTag(4);
        else result = photonCam.getAprilTag(7);
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (result != null) {
            range = result.getBestCameraToTarget().getX();
        }

        if (range <= 3.821 && range >= 1.735) {
            calculatedSetPoint = m_interpolator.get(range);
        } else {
            calculatedSetPoint = m_arm.getArmPosition();
        }

        m_arm.setArmPosition(calculatedSetPoint);
        SmartDashboard.putNumber("Range", range);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("IsAtPosition", m_arm.isAtPosition(calculatedSetPoint));
        return m_arm.isAtPosition(calculatedSetPoint);
    }
}
