// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.utils.CameraDriveUtil;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    // The robot's subsystems

    //Vision Subsystems

    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    //private final Climber m_climber = new Climber(ClimberConstants.kClimberLeftCanId,ClimberConstants.kClimberRightCanId);
    
    private final InterpolatingDoubleTreeMap interpolator = new InterpolatingDoubleTreeMap();

    private final int speakerTagID;

    //Driver and Operator Controllers
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    private final boolean isAllianceRed = m_robotDrive.isAllianceRed();

    public Pose2d currentPose2d = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        if (isAllianceRed) speakerTagID = 4;
        else speakerTagID = 7;

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> //m_robotDrive.driveRobotRelative(new ChassisSpeeds(2.0, 0.0, 0.0)), m_robotDrive));
                        m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY() * (1.0 - m_driverController.getLeftTriggerAxis() * 0.5), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX() * (1.0 - m_driverController.getLeftTriggerAxis() * 0.5), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                !m_driverController.getHID().getLeftBumper(), 
                                false),
                        m_robotDrive));
    }


    private void configureButtonBindings() {
        //Force robot to stop in X formation
        new Trigger(m_driverController.povLeft())
                .whileTrue(new RunCommand(
                        m_robotDrive::setX,
                        m_robotDrive));

        //Reset the Gyro to zero heading with the D-Pad
        new Trigger(m_driverController.povDown())
                .onTrue(new InstantCommand(
                        m_robotDrive::zeroHeading,
                        m_robotDrive));

    }

    public void configureShooterInterpolation() {
        //Put values for shooter angles into interpolation tree
        // Modifying calculated values to fine-tune angles (03/03/2024)
        interpolator.put(1.020, 0.368);
        interpolator.put(1.516, 0.490);
        interpolator.put(2.074, 0.563);
        interpolator.put(2.640, 0.668);
        interpolator.put(3.078, 0.723);
        interpolator.put(4.013, 0.754);
        interpolator.put(5.080, 0.766);
    }
    public CommandXboxController getXboxDriver() {
        return m_driverController;
    }

    public CommandXboxController getXboxOperator() {
        return m_operatorController;
    }

    public DriveSubsystem getDrivetrain() {
        return m_robotDrive;
    }

    public boolean isAllianceRed() {
        return isAllianceRed;
    }

    public void updatePose() {
        currentPose2d = m_robotDrive.getPhotonPose();
    }

    public void initPose() {
        currentPose2d = m_robotDrive.getPhotonPose();
    }
}
