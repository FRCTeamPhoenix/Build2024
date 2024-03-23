// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonClass;

import frc.utils.FireControlUtil;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class cmd_TargetShooterToSpeaker extends Command {
    private final Arm m_arm;
    private final InterpolatingDoubleTreeMap m_treeMap;
    private final DriveSubsystem m_drive;
    private final Pose2d speakerPose;

    private double calculatedSetPoint;

    private double armStartpoint;
    public cmd_TargetShooterToSpeaker(InterpolatingDoubleTreeMap treeMap, Arm arm, DriveSubsystem drive) {
        m_treeMap = treeMap;
        m_arm = arm;
        m_drive = drive;
        armStartpoint = m_arm.getArmPosition();

        if (drive.isAllianceRed()) speakerPose = Constants.VisionConstants.kTagLayout.getTagPose(4).get().toPose2d();
        else
            speakerPose = Constants.VisionConstants.kTagLayout.getTagPose(7).get().toPose2d(); 

        Transform2d transformToSpeaker = m_drive.getPhotonPose().minus(speakerPose);
        double distance = Math.sqrt(Math.pow(transformToSpeaker.getX(), 2) + Math.pow(transformToSpeaker.getY(), 2));
        if (distance <= 4.5754 && distance >= 0.60652) {
            calculatedSetPoint = m_treeMap.get(distance);
        }
        else {
            calculatedSetPoint = m_arm.getArmPosition();
        }

        m_arm.setArmPosition(calculatedSetPoint);

        addRequirements(m_arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Transform2d transformToSpeaker = m_drive.getPhotonPose().minus(speakerPose);
        double distance = Math.sqrt(Math.pow(transformToSpeaker.getX(), 2) + Math.pow(transformToSpeaker.getY(), 2));
        if (distance <= 4.5754 && distance >= 0.60652) {
            calculatedSetPoint = m_treeMap.get(distance);
        }
        else {
            calculatedSetPoint = m_arm.getArmPosition();
        }

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
