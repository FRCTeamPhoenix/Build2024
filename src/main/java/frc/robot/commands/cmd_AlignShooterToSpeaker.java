// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private final CommandXboxController m_controller;
    private final int speakerTagID;

    private double rot;

    private double setpoint;

    public cmd_AlignShooterToSpeaker(DriveSubsystem drive, PhotonClass camera, CommandXboxController controller) {
        m_drive = drive;
        m_cam = camera;
        m_controller = controller;
        if (drive.isAllianceRed()) speakerTagID = 4;
        else speakerTagID = 7;
        addRequirements(m_drive);
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
            rot = -CameraDriveUtil.getDriveRot(target.getYaw(), 0);
        }
        else {
            rot = 0;
        }
        m_drive.drive(0, 0, rot, false, false);
    }

    @Override
    public void end(boolean interrupted) {
        if (General.LOGGING)
            System.out.println("End Move Arm Down");
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(rot) <= 0.005){
            return true;
        }
        return false;
    }
}
