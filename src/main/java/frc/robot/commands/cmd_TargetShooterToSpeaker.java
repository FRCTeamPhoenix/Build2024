// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonClass;

import frc.utils.FireControlUtil;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class cmd_TargetShooterToSpeaker extends Command {
    private final Arm m_arm;
    private final FireControlUtil m_util;
    private final DriveSubsystem m_drive;

    private double calculatedSetPoint;

    private double armStartpoint;
    public cmd_TargetShooterToSpeaker(FireControlUtil util, Arm arm, DriveSubsystem drive) {
        m_util = util;
        m_arm = arm;
        m_drive = drive;
        armStartpoint = m_arm.getArmPosition();

        calculatedSetPoint = m_util.getShooterAngle(m_drive.getPhotonPose(), armStartpoint);
        m_arm.setArmPosition(calculatedSetPoint);

        addRequirements(m_arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        armStartpoint = m_arm.getArmPosition();
        calculatedSetPoint = m_util.getShooterAngle(m_drive.getPhotonPose(), armStartpoint);
        m_arm.setArmPosition(calculatedSetPoint);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_arm.isAtPosition(calculatedSetPoint);
    }
}
