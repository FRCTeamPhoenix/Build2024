// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Constants.General;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OakCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.OakCameraObject;

public class cmd_AlignAndDriveToNote extends Command {
  private DriveSubsystem m_robotDrive;
  private double setpoint;

  //TODO: Tune PID
  private PIDController pid = new PIDController(0.0015, 0.0, 0.0);

  public cmd_AlignAndDriveToNote(DriveSubsystem robotDrive) {
    m_robotDrive = robotDrive;
    addRequirements(m_robotDrive);
    pid.enableContinuousInput(0, 360);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    OakCameraObject nearestNote = OakCamera.findClosestNote();
    
    if (nearestNote == null){
      SmartDashboard.putBoolean("Sees Note", false);
      setpoint = 0.0;
      return;
    }
    else {
      SmartDashboard.putBoolean("Sees Note", true);
      SmartDashboard.putNumber("XAngle", nearestNote.getXAngle());
    }
    SmartDashboard.putNumber("XAngle", nearestNote.getXAngle());

    //Turn based on PID calculation
    setpoint = pid.calculate(nearestNote.getXAngle(), 0.0);
    m_robotDrive.drive(1.5 / Constants.DriveConstants.kMaxSpeedMetersPerSecond,
            0.0, -setpoint, false, false);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Ended command");
    if (General.LOGGING)
      System.out.println("End Drive Nearest Note");
    m_robotDrive.drive(0.0, 0.0, 0.0, false, false);
  }

  @Override
  public boolean isFinished() {
    //Need to run this constantly
    return false;
  }
}
