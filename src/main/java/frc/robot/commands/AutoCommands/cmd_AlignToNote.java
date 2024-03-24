// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import frc.robot.Constants.General;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OakCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.OakCameraObject;
import frc.utils.CameraDriveUtil;

public class cmd_AlignToNote extends Command {
  private DriveSubsystem m_robotDrive;
  private double setpoint;

  public cmd_AlignToNote(DriveSubsystem robotDrive) {
    m_robotDrive = robotDrive;
    addRequirements(m_robotDrive);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    OakCameraObject nearestNote = OakCamera.findClosestNote();
    if (nearestNote == null){
      SmartDashboard.putBoolean("Sees Note", false);
     // setpoint = 0.0;
      //return;
    }
    else {
      SmartDashboard.putBoolean("Sees Note", true);
    }
    // if the robot is not facing the note turn the robot until it is inside the angles of the front camera
    SmartDashboard.putNumber("XAngle", nearestNote.getXAngle());
    setpoint = CameraDriveUtil.getDriveRot(nearestNote.getXAngle(), 0.0);
    m_robotDrive.drive(0.0, 0.0, -setpoint, false, false);
  }

  @Override
  public void end(boolean interrupted) {
    if (General.LOGGING)
      System.out.println("End Drive Nearest Note");
  }

  @Override
  public boolean isFinished() {
    // if the robot is alligned with the note then finish the command
    if (Math.abs(setpoint) <= 0.005){
      return true;
    }
    return false;
  }
}
