// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import frc.robot.Constants.General;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OakCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.OakCameraObject;
import frc.utils.CameraDriveUtil;

public class cmd_AutoDriveNearestNote extends Command {
  private OakCamera m_oakCamera;
  private DriveSubsystem m_robotDrive;
  private boolean noteInCam1;
  private double initialTurn;
  private double xAngle;
  private double distance;
  private double xVelocity;
  private double yVelocity;
  private double thetaVelocity;
  private PathPlannerPath path;

  public cmd_AutoDriveNearestNote(OakCamera oakCamera, DriveSubsystem robotDrive) {
    m_oakCamera = oakCamera;
    m_robotDrive = robotDrive;
    addRequirements(m_oakCamera);
    addRequirements(m_robotDrive);
  }

  @Override
  public void initialize() {
    OakCameraObject nearestNote = OakCamera.findClosestNote();
    if (nearestNote == null) return;
    path = m_robotDrive.generateNotePath(nearestNote);
  }

  @Override
  public void execute() {
    OakCameraObject nearestNote = OakCamera.findClosestNote();
    if (nearestNote == null) return;
    AutoBuilder.followPath(path);
  }

  @Override
  public void end(boolean interrupted) {
    if (General.LOGGING)
      System.out.println("End Drive Nearest Note");
  }

  @Override
  public boolean isFinished() {
    if ( (xVelocity == 0) && (yVelocity == 0) && (thetaVelocity == 0) ) {
      return true;
    } else { 
      return false;
    }
  }
}
