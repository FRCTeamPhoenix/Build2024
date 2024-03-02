// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.utils.FireControlUtil;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.PhotonPose;
import frc.robot.subsystems.PhotonClass;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
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
//    public final PhotonClass frontPhotonCamera = new PhotonClass(VisionConstants.kFrontCameraName, VisionConstants.kFrontTransform);
    public final PhotonClass rearPhotonCamera = new PhotonClass(VisionConstants.kRearCameraName, VisionConstants.kRearTransform);
//    public final PhotonClass leftPhotonCamera = new PhotonClass(VisionConstants., VisionConstants.);

//    public final PhotonPose frontPhotonPose = new PhotonPose(frontPhotonCamera);
    public final PhotonPose rearPhotonPose = new PhotonPose(rearPhotonCamera);
//    public final PhotonPose leftPhotonPose = new PhotonPose();

    public final PhotonPose[] photonPoses = {rearPhotonPose}; // {rearPhotonPose, frontPhotonPose};

    private final DriveSubsystem m_robotDrive = new DriveSubsystem(photonPoses);
    private final Shooter m_shooter = new Shooter(10, 11);
    private final Intake m_intake = new Intake(12);
    private final Arm m_arm = new Arm(13, 14);

    private final InterpolatingDoubleTreeMap interpolator = new InterpolatingDoubleTreeMap();

    private final int speakerTagID;

    //Auto From PathPlanner
    private final SendableChooser<Command> autoChooser;

    //Driver and Operator Controllers
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    private final boolean isAllianceRed = m_robotDrive.isAllianceRed();

    private final FireControlUtil fireControlUtil = new FireControlUtil(isAllianceRed);

    public Pose2d currentPose2d = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
    private Pose2d priorPose2d;

    public RobotContainer() {
        configureShooterInterpolation();

        NamedCommands.registerCommand("cg_StopShootNote", new cg_StopShootNote(m_intake,m_shooter));

        // Configure the button bindings
        configureButtonBindings();

        //NamedCommands.registerCommand("cg_ShootNote", new cg_ShootNote(m_intake, m_shooter));
        //NamedCommands.registerCommand("cg_StopShootNote", new cg_StopShootNote(m_intake, m_shooter));

        autoChooser = AutoBuilder.buildAutoChooser("test_auto");

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
                                getRobotRotation(m_driverController.getHID().getYButton()),
                                true, m_driverController.getHID().getRightBumper()),
                        m_robotDrive));
    }


    private void configureButtonBindings() {
        //Force robot to stop in X formation
        new Trigger(m_driverController.x())
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.setX(),
                        m_robotDrive));

        //Reset the Gyro to zero heading with B
        new Trigger(m_driverController.b())
                .onTrue(new RunCommand(
                        () -> m_robotDrive.zeroHeading(),
                        m_robotDrive));

        //Move Arm To subwoofer shooting
        final Trigger btn_op_X = new Trigger(m_operatorController.x());
        btn_op_X.onTrue(new cmd_MoveArmToPosition(.6, 1, m_arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        //Move Arm To Amp shoot
        final Trigger btn_op_B = new Trigger(m_operatorController.b());
        btn_op_B.onTrue(new cmd_MoveArmToPosition(3.11, 1, m_arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        //Move Arm Up
        final Trigger btn_op_Y = new Trigger(m_operatorController.y());
        btn_op_Y.whileTrue(new cmd_MoveArmUp(m_arm, ArmConstants.ArmMoveSetPoint).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));

        //Move Arm Down
        final Trigger btn_op_A = new Trigger(m_operatorController.a());
        btn_op_A.whileTrue(new cmd_MoveArmDown(m_arm, ArmConstants.ArmMoveSetPoint).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));

        //Move Arm To speaker shooting based on distance
        final Trigger btn_drv_Y = new Trigger(m_driverController.y());
        btn_drv_Y.onTrue(new cmd_TargetShooterToSpeaker(fireControlUtil, m_arm, m_robotDrive).withInterruptBehavior(InterruptionBehavior.kCancelSelf));


        Trigger povUpPressed = m_operatorController.povUp();
        povUpPressed.whileTrue(new cmd_MoveArmUp(m_arm, .05).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));
      
        Trigger povLeftPressed = m_operatorController.povLeft();
        povLeftPressed.whileTrue(new cg_FloorIntake(m_intake, m_arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));
       
        Trigger povDownPressed = m_operatorController.povDown();
        povDownPressed.whileTrue(new cmd_MoveArmDown(m_arm, .05).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));

        m_operatorController.rightTrigger(.5).whileTrue(new cg_ShootNote(m_intake, m_shooter)).whileFalse(new cg_StopShootNote(m_intake, m_shooter));
        m_operatorController.leftBumper().whileTrue(new cmd_IntakeNote(m_intake)).whileFalse(new cmd_StopIntake(m_intake));

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

    public double getRobotRotation(boolean alignToSpeaker) {
        PhotonTrackedTarget target = rearPhotonCamera.getAprilTag(speakerTagID);
        if (target != null && alignToSpeaker) return -CameraDriveUtil.getDriveRot(target.getYaw(), 0);
        else return -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband);
    }

    public double getRobotRotationFeedForward(boolean alignToSpeaker) {
        PhotonTrackedTarget target = rearPhotonCamera.getAprilTag(speakerTagID);
        if (target != null && alignToSpeaker) return -CameraDriveUtil.getDriveRotWithFeedForward(target.getYaw(), 0, priorPose2d, currentPose2d, isAllianceRed);
        else return -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband);
    }

    public double getRobotRotationFireControl(boolean alignToSpeaker) {
        if (alignToSpeaker) return -fireControlUtil.getPIDValue(currentPose2d, m_robotDrive.getRotation());
        else return -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband);
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
