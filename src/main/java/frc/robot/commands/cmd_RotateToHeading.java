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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class cmd_RotateToHeading extends Command {
    private final DriveSubsystem m_drive;
    private final PIDController pidController;
    private final CommandXboxController m_controller;
    private final int speakerTagID;
    private final double desiredAngle;

    private double rot;

    private boolean isAutonomous;
    private double setpoint;

    public cmd_RotateToHeading(DriveSubsystem drive, PhotonClass camera, CommandXboxController controller, double desiredHeading) {
        m_drive = drive;

        desiredAngle = desiredHeading;

        pidController = new PIDController(0.015, 0.0, 0.0);
        pidController.enableContinuousInput(0, 360);

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
        rot = pidController.calculate(m_drive.getRotation().getDegrees(), desiredAngle);

        m_drive.drive(
            -MathUtil.applyDeadband(m_controller.getLeftY() * (1.0 - m_controller.getLeftTriggerAxis() * 0.5), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_controller.getLeftX() * (1.0 - m_controller.getLeftTriggerAxis() * 0.5), OIConstants.kDriveDeadband),
            rot,
            true, m_controller.getHID().getLeftBumper());
    }

    @Override
    public void end(boolean interrupted) {
        if (General.LOGGING)
            System.out.println("End Move Arm Down");
    }

    @Override
    public boolean isFinished() {
        if (rot <= 0.005){
            return true;
        }
        return false;
    }
}
