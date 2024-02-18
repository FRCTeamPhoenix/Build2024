// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.OakCamera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.PhotonPose;
import frc.robot.subsystems.PhotonClass;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  // The robot's subsystems
  public final OakCamera m_OakCamera = new OakCamera();
  public final LimeLight m_frontLimeLight = new LimeLight("limelight-front");

  public final LimeLight m_rearLimeLight = new LimeLight("limelight-rear");

  public final PhotonClass photonCamera = new PhotonClass(VisionConstants.kCameraName, VisionConstants.kRobotToCam);
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(photonCamera);
  private final Shooter m_shooter = new Shooter(10, 11);
  private final Intake m_intake = new Intake(12);
  private final Arm m_arm = new Arm(13, 14);

  //Vision Subsystems
  public final PhotonPose vision = m_robotDrive.getPhotonPose();
  
  //Auto From PathPlanner
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("path1");

  //Driver and Operator Controllers
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  CommandXboxController m_op_command = new CommandXboxController(OIConstants.kOperatorControllerPort);


  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //Add Autos
    SmartDashboard.putData("Auto", autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY() * (1.0 - m_driverController.getLeftTriggerAxis() * 0.5), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX() * (1.0 - m_driverController.getLeftTriggerAxis() * 0.5), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, m_driverController.getRightBumper()),
            m_robotDrive));
  }


  private void configureButtonBindings() {
    //Force robot to stop in X formation
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    //Reset the Gyro to zero heading with the Right Bumper
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .onTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    //Move Arm To Speaker shoot
    final JoystickButton btn_op_X = new JoystickButton(m_operatorController, XboxController.Button.kX.value);        
    btn_op_X.onTrue(new cmd_MoveArmToPosition(.7,1,m_arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf));    

    //Move Arm To Amp shoot
    final JoystickButton btn_op_B = new JoystickButton(m_operatorController, XboxController.Button.kB.value);        
    btn_op_B.onTrue(new cmd_MoveArmToPosition(3.11,1,m_arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf));    

    //Move Arm Up
    final JoystickButton btn_op_Y = new JoystickButton(m_operatorController, XboxController.Button.kY.value);        
    btn_op_Y.whileTrue(new cmd_MoveArmUp(m_arm,ArmConstants.ArmMoveSetPoint).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));    

    //Move Arm Down
    final JoystickButton btn_op_A = new JoystickButton(m_operatorController, XboxController.Button.kA.value);        
    btn_op_A.whileTrue(new cmd_MoveArmDown(m_arm,ArmConstants.ArmMoveSetPoint).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));    

    POVButton povUpPressed = new POVButton(m_operatorController, 0);
    povUpPressed.whileTrue(new cmd_MoveArmUp(m_arm,.05).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));

    POVButton povDownPressed = new POVButton(m_operatorController, 180);
    povDownPressed.whileTrue(new cmd_MoveArmDown(m_arm,.05).withInterruptBehavior(InterruptionBehavior.kCancelSelf)).whileFalse(new cmd_StopArm(m_arm));

    m_op_command.rightTrigger(.5).whileTrue(new cg_ShootNote(m_intake,m_shooter)).whileFalse(new cg_StopShootNote(m_intake, m_shooter));
    m_op_command.leftBumper().whileTrue(new cmd_LoadNote(m_intake)).whileFalse(new cmd_StopIntake(m_intake));
    m_op_command.leftTrigger(.5).whileTrue(new cmd_EjectNote(m_intake)).whileFalse(new cmd_StopIntake(m_intake));

  }

  

  //All Getters/Setters for the robot objects.
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public LimeLight getFrontLimeLight() {
    return m_frontLimeLight;
  }

  public LimeLight getRearLimeLight() {
    return m_rearLimeLight;
  }

  public XboxController getXboxDriver() {
    return m_driverController;
  }

  public XboxController getXboxOperator() {
    return m_operatorController;
  }

  public DriveSubsystem getDrivetrain(){
    return m_robotDrive;
  }

  public Arm getArm(){
    return m_arm;
  }

  public Shooter getShooter(){
    return m_shooter;
  }

  public Intake getIntake(){
    return m_intake;
  }
  public PhotonPose getPhotonPose(){
    return vision;
  }
}
