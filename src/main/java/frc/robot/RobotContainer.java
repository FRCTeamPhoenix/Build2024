// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.commands.AutoCommands.cg_AutoIntakeToFloor;
import frc.robot.commands.AutoCommands.cg_AutoNotePickup;
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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

import frc.utils.FireControlUtil;

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
        NamedCommands.registerCommand("cg_AutoIntakeToFloor", new cg_AutoIntakeToFloor(m_intake, m_arm));
        NamedCommands.registerCommand("cg_AutoNotePickup", new cg_AutoNotePickup(m_intake, m_arm, firstOakCamera, m_robotDrive));
        NamedCommands.registerCommand("cg_ShootAndMoveArm", new cg_ShootAndMoveArm(interpolator, m_arm, m_robotDrive, rearPhotonCamera, m_shooter, m_intake, m_driverController));
        NamedCommands.registerCommand("cg_StopShootNote", new cg_StopShootNote(m_intake,m_shooter));
        NamedCommands.registerCommand("cmd_LowerArm", new cmd_MoveArmToPosition(0.1, 1, m_arm).withTimeout(1));
        NamedCommands.registerCommand("shoot", new cg_ShootNote(m_intake, m_shooter));
        NamedCommands.registerCommand("cg_FloorIntake", new cg_FloorIntake(m_intake, m_arm));
        NamedCommands.registerCommand("cmd_TargetShooterToSpeaker", new cmd_TargetShooterToSpeaker(interpolator, m_arm, m_robotDrive));
        NamedCommands.registerCommand("cmd_AlignShooterToSpeaker", new cmd_AlignShooterToSpeaker(m_robotDrive, rearPhotonCamera, m_driverController));

        SmartDashboard.putData("cg_moveArm + shoot", NamedCommands.getCommand("cg_ShootAndMoveArm"));
        SmartDashboard.putData("cg stop shoot", NamedCommands.getCommand("cg_StopShootNote"));
        SmartDashboard.putData("cmd_lowerarm", NamedCommands.getCommand("cmd_LowerArm"));
        SmartDashboard.putData("align", NamedCommands.getCommand("align"));
        SmartDashboard.putData("Shoot", NamedCommands.getCommand("shoot"));
        SmartDashboard.putData("cg_AutoNotePickup", NamedCommands.getCommand("cg_AutoNotePickup"));
        SmartDashboard.putString("color", "teamColor");

        // Configure the button bindings
        configureButtonBindings();

        autoChooser = AutoBuilder.buildAutoChooser("macmahon");

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
        btn_op_B.onTrue(new cmd_MoveArmToPosition(3.0, 1, m_arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

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
        povUpPressed.whileTrue(new cmd_Climber(m_climber, 8).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_Climber(m_climber, 0.0));
        // povUpPressed.whileTrue(new cmd_MoveArmUp(m_arm, .05).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));

        Trigger povLeftPressed = m_operatorController.povLeft();
        povLeftPressed.toggleOnTrue(new cmd_SpinShooter(m_shooter, m_robotDrive, m_arm, m_intake).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).toggleOnFalse(new cmd_StopShoot(m_shooter));
        // povLeftPressed.whileTrue(new cg_FloorIntake(m_intake, m_arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));

        Trigger povDownPressed = m_operatorController.povDown();
        povDownPressed.whileTrue(new cmd_Climber(m_climber, -8).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_Climber(m_climber, 0.0));
        // povDownPressed.whileTrue(new cmd_MoveArmDown(m_arm, .05).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));

        Trigger povRightPressed = m_operatorController.povRight();
        povRightPressed.toggleOnTrue(new cmd_MoveArmToPosition(ArmConstants.ARM_MIN_ANGLE, 1, m_arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        //Operator Bumpers and Triggers
        m_operatorController.rightTrigger(.5).whileTrue(new cg_ShootNote(m_intake, m_shooter)).whileFalse(new cg_StopShootNote(m_intake, m_shooter));

        m_operatorController.leftBumper().whileTrue(new cmd_IntakeNote(m_intake)).whileFalse(new cmd_StopIntake(m_intake));

        m_operatorController.rightBumper().whileTrue(new cmd_ManualIntake(m_intake)).whileFalse(new cmd_StopIntake(m_intake));

        m_operatorController.leftTrigger(.5).whileTrue(new cmd_EjectNote(m_intake)).whileFalse(new cmd_StopIntake(m_intake));

        //Cardinal Directions
        m_driverController.a().whileTrue(new cmd_RotateToHeading(m_robotDrive, frontPhotonCamera, m_driverController, 0).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        m_driverController.y().whileTrue(new cmd_RotateToHeading(m_robotDrive, frontPhotonCamera, m_driverController, 180).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        m_driverController.b().whileTrue(new cmd_RotateToHeading(m_robotDrive, frontPhotonCamera, m_driverController, -90).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        m_driverController.x().whileTrue(new cmd_RotateToHeading(m_robotDrive, frontPhotonCamera, m_driverController, 90).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    }

    public void configureShooterInterpolation() {
        //Put values for shooter angles into interpolation tree
        // Modifying calculated values to fine-tune angles (03/03/2024)
        interpolator.put(0.61, 0.364);
        interpolator.put(1.63, 0.514);
        interpolator.put(2.35, 0.674);
        interpolator.put(2.85, 0.724);
        interpolator.put(3.40, 0.794);
        interpolator.put(3.92, 0.804);
        interpolator.put(4.58, 0.834);
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
