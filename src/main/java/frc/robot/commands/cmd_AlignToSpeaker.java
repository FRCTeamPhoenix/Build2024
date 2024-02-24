// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonClass;
import frc.utils.CameraDriveUtil;

import org.photonvision.PhotonUtils;
import java.lang.Math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class cmd_AlignToSpeaker extends Command {
  private final DriveSubsystem m_drive;
  private final PhotonClass m_camera;
  private final CommandXboxController m_driverController;
  private final int speakerTag;

  /**
   * Command the arm to move to a setpoint.
   *
   * @param interpolator Interpolating Tree Map for shooter vaules
   * @param speed The speed the arm will move
   * @param arm The Arm Subsystem
   */
  public cmd_AlignToSpeaker(DriveSubsystem drive, PhotonClass camera, CommandXboxController controller) {
    m_camera = camera;
    m_drive = drive;
    m_driverController = controller;

    if (drive.isAllianceRed()) speakerTag = 4;
    else speakerTag = 7;

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    m_drive.drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY() * (1.0 - m_driverController.getLeftTriggerAxis() * 0.5), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX() * (1.0 - m_driverController.getLeftTriggerAxis() * 0.5), OIConstants.kDriveDeadband),
        CameraDriveUtil.getDriveRot(m_camera.getAprilTag(speakerTag).getYaw(), 0),
        true, m_driverController.getHID().getRightBumper());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    double thetaError = 0 - m_camera.getAprilTag(speakerTag).getYaw();
    if (Math.abs(thetaError) < 5) {
      thetaError = 0;
    }
    return thetaError == 0;
  }
}
