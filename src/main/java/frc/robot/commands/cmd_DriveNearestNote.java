// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.General;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OakCamera;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.OakCameraObject;
import frc.utils.CameraDriveUtil;

public class cmd_DriveNearestNote extends Command {
  private OakCamera m_oakCamera;
  private DriveSubsystem m_robotDrive;
  private boolean noteInCam1;
  private double initialTurn;
  private double xAngle;
  private double distance;
  private double xVelocity;
  private double yVelocity;
  private double thetaVelocity;

  public cmd_DriveNearestNote(OakCamera oakCamera, DriveSubsystem robotDrive) {
    m_oakCamera = oakCamera;
    m_robotDrive = robotDrive;
    addRequirements(m_oakCamera);
    addRequirements(m_robotDrive);
  }

  @Override
  public void initialize() {
    OakCameraObject nearestNote = OakCamera.findClosestNote();
    if ( (nearestNote.getXAngle() >= 30) && (nearestNote.getXAngle() <= 330) ) {
      noteInCam1 = false;
      initialTurn = nearestNote.getXAngle() >= 180 ? nearestNote.getXAngle() : - ( 360 - nearestNote.getXAngle() );

    }
    else{
      noteInCam1 = true;
    }
  }

  @Override
  public void execute() {
    OakCameraObject nearestNote = OakCamera.findClosestNote();
    if (noteInCam1 == false) {
      m_robotDrive.drive(0, 0, CameraDriveUtil.getDriveRot(initialTurn, 0), false, false);
      if ( (nearestNote.getXAngle() <= 30) || (nearestNote.getXAngle() >= 330) ) {
        noteInCam1 = true;
      }
    }
    xAngle = nearestNote.getXAngle();
    distance = nearestNote.getHorizontalDistance();
    xVelocity = CameraDriveUtil.getDriveX(xAngle, distance, 100);
    yVelocity = CameraDriveUtil.getDriveY(xAngle, distance, 100);
    thetaVelocity = CameraDriveUtil.getDriveRot(xAngle, 0);
    m_robotDrive.drive( xVelocity, yVelocity, thetaVelocity, false, false);
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
