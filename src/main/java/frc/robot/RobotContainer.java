// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.commands.AutoCommands.cg_AutoIntakeToFloor;
import frc.robot.commands.AutoCommands.cg_AutoNotePickup;
import frc.robot.commands.AutoCommands.cmd_AlignAndDriveToNote;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.photonvision.targeting.PhotonTrackedTarget;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.PhotonClass;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OakCamera;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
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
    public final PhotonClass frontPhotonCamera = new PhotonClass(VisionConstants.kFrontCameraName, VisionConstants.kFrontTransform);
    public final PhotonClass rearPhotonCamera = new PhotonClass(VisionConstants.kRearCameraName, VisionConstants.kRearTransform);
    public final PhotonClass leftPhotonCamera = new PhotonClass(VisionConstants.kLeftCameraName, VisionConstants.kLeftTransform);
    public final PhotonClass rightPhotonCamera = new PhotonClass(VisionConstants.kRightCameraName, VisionConstants.kRightTransform);

    public final OakCamera firstOakCamera = new OakCamera();

    public final PhotonClass[] photonCams = {frontPhotonCamera, rearPhotonCamera, leftPhotonCamera, rightPhotonCamera};

    private final DriveSubsystem m_robotDrive = new DriveSubsystem(photonCams);
    private final Shooter m_shooter = new Shooter(10, 11);
    private final Intake m_intake = new Intake(12);
    private final Arm m_arm = new Arm(13, 14);
    private final Climber m_climber = new Climber(ClimberConstants.kClimberLeftCanId,ClimberConstants.kClimberRightCanId);
    
    private final InterpolatingDoubleTreeMap interpolator = new InterpolatingDoubleTreeMap();

    private final int speakerTagID;

    //Auto From PathPlanner
    private final SendableChooser<Command> autoChooser;

    //Driver and Operator Controllers
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    private final boolean isAllianceRed = m_robotDrive.isAllianceRed();

    public Pose2d currentPose2d = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

    public RobotContainer() {
        configureShooterInterpolation();

        NamedCommands.registerCommand("Shoot", new cg_AutoShootNote(interpolator, m_arm, m_robotDrive, m_intake, m_shooter));
        NamedCommands.registerCommand("ShootSlow", new cg_AutoSlowShootNote(interpolator, m_arm, m_robotDrive, m_intake, m_shooter));
        NamedCommands.registerCommand("DriveAndIntake", new cg_AutoNotePickup(m_intake, m_robotDrive).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        NamedCommands.registerCommand("RotateToSpeaker", new cmd_AlignShooterToSpeaker(m_robotDrive, rearPhotonCamera, m_driverController));
        NamedCommands.registerCommand("PathToShoot", new cg_DriveToShootPos(m_robotDrive));
        NamedCommands.registerCommand("ShooterToSpeaker", new cmd_TargetShooterToSpeaker(interpolator, m_arm, m_robotDrive));
        NamedCommands.registerCommand("FloorIntake", new cg_AutoIntakeToFloor(m_intake, m_arm));
        NamedCommands.registerCommand("ShootAndMoveArm", new cg_ShootAndMoveArm(interpolator, m_arm, m_robotDrive, frontPhotonCamera, m_shooter, m_intake, m_driverController));
        NamedCommands.registerCommand("SubwooferShoot", new cg_SubwooferShoot(m_arm, m_robotDrive, m_intake, m_shooter));
        NamedCommands.registerCommand("SpinMotors", new cmd_SpinShootMotors(m_shooter));

        SmartDashboard.putString("Color", "teamColor");
        SmartDashboard.putNumber("Turnspeed", 0.002);
        SmartDashboard.putNumber("Movespeed", 1.5);
        SmartDashboard.putNumber("Turnsetpoint", 0.0025);


        SmartDashboard.putData("aimArm", new cmd_TargetShooterToSpeaker(interpolator, m_arm, m_robotDrive));
        SmartDashboard.putData("cg_shootNote", new cg_AutoSlowShootNote(interpolator, m_arm, m_robotDrive, m_intake, m_shooter));
        SmartDashboard.putData("stopShoot", new cmd_StopShoot(m_shooter));
        SmartDashboard.putData("stopIntake", new cmd_StopIntake(m_intake));
        SmartDashboard.putData("alignToNote", new cmd_AlignAndDriveToNote(m_robotDrive).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        SmartDashboard.putData("Drive and Intake Note", new cg_AutoNotePickup(m_intake, m_robotDrive).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        SmartDashboard.putData("RotateToSpeaker", new cmd_AlignShooterToSpeaker(m_robotDrive, rearPhotonCamera, m_driverController));
        SmartDashboard.putData("PathToShoot", new cg_DriveToShootPos(m_robotDrive));
        

        // Configure the button bindings
        configureButtonBindings();

        autoChooser = AutoBuilder.buildAutoChooser("twonote_right");

        //Add Autos
        SmartDashboard.putData("Auto", autoChooser);

        if (isAllianceRed) speakerTagID = 4;
        else speakerTagID = 7;

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                    () -> m_robotDrive.drive(
                        -MathUtil.applyDeadband(m_driverController.getLeftY() * (1.0 - m_driverController.getLeftTriggerAxis() * 0.5), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverController.getLeftX() * (1.0 - m_driverController.getLeftTriggerAxis() * 0.5), OIConstants.kDriveDeadband),
                        getRobotRotation(),
                        true, m_driverController.getHID().getLeftBumper()),
                    m_robotDrive));
        
                // new RunCommand(
                //         () -> //m_robotDrive.driveRobotRelative(new ChassisSpeeds(2.0, 0.0, 0.0)), m_robotDrive));
                //         m_robotDrive.drive(
                //                 -MathUtil.applyDeadband(m_driverController.getLeftY() * (1.0 - m_driverController.getLeftTriggerAxis() * 0.5), OIConstants.kDriveDeadband),
                //                 0.0, //-MathUtil.applyDeadband(m_driverController.getLeftX() * (1.0 - m_driverController.getLeftTriggerAxis() * 0.5), OIConstants.kDriveDeadband),
                //                 getRobotRotation(),
                //                 true, m_driverController.getHID().getLeftBumper()),
                //         m_robotDrive));

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

        //Move Arm To subwoofer shooting
        final Trigger btn_op_X = new Trigger(m_operatorController.x());
        btn_op_X.onTrue(new cmd_MoveArmToPosition(.436, 1, m_arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        //Move Arm To Amp shoot
        final Trigger btn_op_B = new Trigger(m_operatorController.b());
        btn_op_B.onTrue(new cmd_MoveArmToPosition(ArmConstants.ARM_MAX_ANGLE, 1, m_arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        //Move Arm Up
        final Trigger btn_op_Y = new Trigger(m_operatorController.y());
        btn_op_Y.whileTrue(new cmd_MoveArmUp(m_arm, ArmConstants.ArmMoveSetPoint).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));

        //Move Arm Down
        final Trigger btn_op_A = new Trigger(m_operatorController.a());
        btn_op_A.whileTrue(new cmd_MoveArmDown(m_arm, ArmConstants.ArmMoveSetPoint).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));

        //Move Arm To speaker shooting based on distance
        Trigger btn_drv_Y = new Trigger(m_driverController.rightBumper());
        btn_drv_Y.whileTrue(new cmd_TargetShooterToSpeaker(interpolator, m_arm, m_robotDrive).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        //Operator Dpad
        Trigger povUpPressed = m_operatorController.povUp();
        povUpPressed.whileTrue(new cmd_Climber(m_climber, 12).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_Climber(m_climber, 0.0));
        // povUpPressed.whileTrue(new cmd_MoveArmUp(m_arm, .05).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));

        Trigger povLeftPressed = m_operatorController.povLeft();
        povLeftPressed.toggleOnTrue(new cmd_SpinShooter(m_shooter, m_robotDrive, m_arm, m_intake).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // povLeftPressed.whileTrue(new cg_FloorIntake(m_intake, m_arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));

        Trigger povDownPressed = m_operatorController.povDown();
        povDownPressed.whileTrue(new cmd_Climber(m_climber, -12).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_Climber(m_climber, 0.0));
        // povDownPressed.whileTrue(new cmd_MoveArmDown(m_arm, .05).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));

        Trigger povRightPressed = m_operatorController.povRight();
        povRightPressed.toggleOnTrue(new cmd_MoveArmToPosition(ArmConstants.ARM_MIN_ANGLE, 1, m_arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        //Operator Bumpers and Triggers
        m_operatorController.rightTrigger(.5).whileTrue(new cg_ShootNote(m_intake, m_shooter)).whileFalse(new cg_StopShootNote(m_intake, m_shooter));

        m_operatorController.leftBumper().whileTrue(new cmd_IntakeNote(m_intake)).onFalse(new cmd_StopIntake(m_intake));

        m_operatorController.rightBumper().whileTrue(new cmd_ManualIntake(m_intake)).onFalse(new cmd_StopIntake(m_intake));

        m_operatorController.leftTrigger(.5).whileTrue(new cmd_EjectNote(m_intake)).onFalse(new cmd_StopIntake(m_intake));

        //Cardinal Directions
        m_driverController.a().whileTrue(new cmd_RotateToHeading(m_robotDrive, m_driverController, 180).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        m_driverController.y().whileTrue(new cmd_RotateToHeading(m_robotDrive, m_driverController, 0).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        m_driverController.b().whileTrue(new cmd_RotateToHeading(m_robotDrive, m_driverController, 90).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        m_driverController.x().whileTrue(new cmd_RotateToHeading(m_robotDrive, m_driverController, -90).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        m_driverController.povRight().whileTrue(new cg_AutoNotePickup(m_intake, m_robotDrive).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).onFalse(new cmd_StopIntake(m_intake));

    }

    public void configureShooterInterpolation() {
        //Put values for shooter angles into interpolation tree
        // Modifying calculated values to fine-tune angles (03/03/2024)
        interpolator.put(1.06, 0.3863);
        interpolator.put(1.46, 0.5149);
        interpolator.put(1.98, 0.5824);
        interpolator.put(2.51, 0.6744);
        interpolator.put(3.32, 0.7605);
        interpolator.put(3.82, 0.7850);
        interpolator.put(4.33, 0.7855); // 
        interpolator.put(5.33, 0.8276);
    }

    //All Getters/Setters for the robot objects.

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
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

    public Arm getArm() {
        return m_arm;
    }

    public Shooter getShooter() {
        return m_shooter;
    }

    public Climber getClimber() {
        return m_climber;
    }

    public double getRobotRotation() {
        boolean alignToSpeaker = m_driverController.rightBumper().getAsBoolean();

        PhotonTrackedTarget rear = rearPhotonCamera.getAprilTag(speakerTagID);
        if (rear != null && alignToSpeaker) return -CameraDriveUtil.getDriveRot(rear.getYaw(), 0);
        else {
            return -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband);
        }
    }

    public Intake getIntake() {
        return m_intake;
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
