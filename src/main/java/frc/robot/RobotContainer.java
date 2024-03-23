// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
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

import frc.utils.FireControlUtil;
import frc.utils.NotePoseGenerator;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.Util;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.PhotonPose;
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

    public final PhotonPose frontPhotonPose = new PhotonPose(frontPhotonCamera);
    public final PhotonPose rearPhotonPose = new PhotonPose(rearPhotonCamera);
    public final PhotonPose leftPhotonPose = new PhotonPose(leftPhotonCamera);
    public final PhotonPose rightPhotonPose = new PhotonPose(rightPhotonCamera);


    public final PhotonPose[] photonPoses = {frontPhotonPose, rearPhotonPose, leftPhotonPose, rightPhotonPose};

    private final DriveSubsystem m_robotDrive = new DriveSubsystem(photonPoses);
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
    private final FireControlUtil fireControlUtil = new FireControlUtil(isAllianceRed);
    private final PIDController turningPID = new PIDController(0.01, 0.0, 0.0);
    public Pose2d currentPose2d = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
    private Pose2d priorPose2d;

    public RobotContainer() {
        configureShooterInterpolation();
        NamedCommands.registerCommand("cg_AutoIntakeToFloor", new cg_AutoIntakeToFloor(m_intake, m_arm));
        NamedCommands.registerCommand("cg_AutoNotePickup", new cg_AutoNotePickup(m_intake, m_arm, firstOakCamera, m_robotDrive));
        NamedCommands.registerCommand("cg_ShootAndMoveArm", new cg_ShootAndMoveArm(fireControlUtil, m_arm, m_robotDrive, rearPhotonCamera, m_shooter, m_intake, m_driverController));
        NamedCommands.registerCommand("cg_StopShootNote", new cg_StopShootNote(m_intake,m_shooter));
        NamedCommands.registerCommand("cmd_LowerArm", new cmd_MoveArmToPosition(0.1, 1, m_arm).withTimeout(1));
        NamedCommands.registerCommand("shoot", new cg_ShootNote(m_intake, m_shooter));
        NamedCommands.registerCommand("cg_FloorIntake", new cg_FloorIntake(m_intake, m_arm));
        NamedCommands.registerCommand("cg_FetchNoteAndShoot", new cg_FetchNoteAndShoot(m_intake,m_shooter,m_robotDrive, m_arm, rearPhotonCamera, firstOakCamera, fireControlUtil, m_driverController));
        NamedCommands.registerCommand("cmd_TargetShooterToSpeaker", new cmd_TargetShooterToSpeaker(fireControlUtil, m_arm, m_robotDrive));
        NamedCommands.registerCommand("cmd_AlignShooterToSpeaker", new cmd_AlignShooterToSpeaker(m_robotDrive, rearPhotonCamera, m_driverController));

        SmartDashboard.putData("cg_moveArm + shoot", NamedCommands.getCommand("cg_ShootAndMoveArm"));
        SmartDashboard.putData("cg stop shoot", NamedCommands.getCommand("cg_StopShootNote"));
        SmartDashboard.putData("cmd_lowerarm", NamedCommands.getCommand("cmd_LowerArm"));
        SmartDashboard.putData("align", NamedCommands.getCommand("align"));
        SmartDashboard.putData("Shoot", NamedCommands.getCommand("shoot"));
        SmartDashboard.putData("cg_AutoNotePickup", NamedCommands.getCommand("cg_AutoNotePickup"));

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
        final Trigger btn_drv_Y = new Trigger(m_driverController.rightBumper());
        btn_drv_Y.whileTrue(new cmd_TargetShooterToSpeaker(fireControlUtil, m_arm, m_robotDrive).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        Trigger povUpPressed = m_operatorController.povUp();
        povUpPressed.whileTrue(new cmd_Climber(m_climber, 8).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_Climber(m_climber, 0.0));
        // povUpPressed.whileTrue(new cmd_MoveArmUp(m_arm, .05).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));

        Trigger povLeftPressed = m_operatorController.povLeft();
        povLeftPressed.toggleOnTrue(new cmd_SpinShooter(m_shooter, m_robotDrive, m_arm, m_intake).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // povLeftPressed.whileTrue(new cg_FloorIntake(m_intake, m_arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));

        Trigger povDownPressed = m_operatorController.povDown();
        povDownPressed.whileTrue(new cmd_Climber(m_climber, -8).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_Climber(m_climber, 0.0));
        // povDownPressed.whileTrue(new cmd_MoveArmDown(m_arm, .05).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));

        m_operatorController.rightTrigger(.5).whileTrue(new cg_ShootNote(m_intake, m_shooter)).whileFalse(new cg_StopShootNote(m_intake, m_shooter));
        m_operatorController.leftBumper().whileTrue(new cmd_IntakeNote(m_intake)).whileFalse(new cmd_StopIntake(m_intake));

        m_operatorController.rightBumper().whileTrue(new cmd_ManualIntake(m_intake)).whileFalse(new cmd_StopIntake(m_intake));

        Trigger povRightPressed = m_operatorController.povRight();
        povRightPressed.toggleOnTrue(new cmd_MoveArmToPosition(ArmConstants.ARM_MIN_ANGLE, 1, m_arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        m_operatorController.leftTrigger(.5).whileTrue(new cmd_EjectNote(m_intake)).whileFalse(new cmd_StopIntake(m_intake));
    }

    public void configureShooterInterpolation() {
        //Put values for shooter angles into interpolation tree
        interpolator.put(0.60652, 0.3497);
        interpolator.put(1.6333, 0.4969);
        interpolator.put(2.3544, 0.6624);
        interpolator.put(2.8532, 0.7114);
        interpolator.put(3.3961, 0.7788);
        interpolator.put(3.9187, 0.7791);
        interpolator.put(4.5754, 0.8218);
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
        boolean alignForward = m_driverController.y().getAsBoolean();
        boolean alignBack = m_driverController.a().getAsBoolean();
        boolean alignRight = m_driverController.b().getAsBoolean();
        boolean alignLeft = m_driverController.x().getAsBoolean();

        if (alignToSpeaker) {
            //PhotonTrackedTarget rear = rearPhotonCamera.getAprilTag(speakerTagID);
            //if (rear != null) return -CameraDriveUtil.getDriveRot(rear.getYaw(), 0);
            return fireControlUtil.getPIDValue(currentPose2d, m_robotDrive.getRotation());
        }
        else if (alignForward) {
            return fireControlUtil.turnToDirection(m_robotDrive.getRotation(), 0);
        }
        else if (alignBack) {
            return fireControlUtil.turnToDirection(m_robotDrive.getRotation(), 180);
        }
        else if (alignRight) {
            return fireControlUtil.turnToDirection(m_robotDrive.getRotation(), -90);
        }
        else if (alignLeft) {
            return fireControlUtil.turnToDirection(m_robotDrive.getRotation(), 90);
        }
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
        priorPose2d = currentPose2d;
        currentPose2d = m_robotDrive.getPhotonPose();
    }

    public void initPose() {
        currentPose2d = m_robotDrive.getPhotonPose();
        priorPose2d = currentPose2d;
    }
}
