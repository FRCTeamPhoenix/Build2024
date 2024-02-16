// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import statements commented for potential future use.
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.OakCamera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

  public final PhotonPose vision = m_robotDrive.getPhotonPose();

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("path1");

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);


  private final Shooter m_shooter = new Shooter(10, 11);
  private final Intake m_intake = new Intake(12);
  private final Arm m_arm = new Arm(13, 14);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
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
    m_arm.setDefaultCommand(new RunCommand(
      () -> m_arm.holdPosition()));

        
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
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
    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .onTrue(new RunCommand(
            () -> m_arm.moveArmUp(),
            m_arm)); 
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .onTrue(new RunCommand(
            () -> m_arm.moveArmDown(),
            m_arm));       
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
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
