// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.General;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonClass;
import frc.utils.CameraDriveUtil;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class cmd_AlignShooterToSpeaker extends Command {
    private final DriveSubsystem m_drive;
    private final PhotonClass m_cam;
    private final int speakerTagID;
    private double setpoint;

    public cmd_AlignShooterToSpeaker(DriveSubsystem drive, PhotonClass camera) {
        m_drive = drive;
        m_cam = camera;
        if (drive.isAllianceRed()) speakerTagID = 4;
        else speakerTagID = 7;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        PhotonTrackedTarget target = null;
        if (m_cam.getCamera().isConnected()) {
            target = m_cam.getAprilTag(speakerTagID);
        }
        if (target != null) {
            double rot = -CameraDriveUtil.getDriveRot(target.getYaw(), 0);
            m_drive.drive(0, 0, rot, true, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (General.LOGGING)
            System.out.println("End Move Arm Down");
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(setpoint) <= 0.005){
            return true;
        }
        return false;
    }
}
