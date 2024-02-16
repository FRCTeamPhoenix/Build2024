// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm;
import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.XboxController;
// import frc.robot.Constants.OIConstants;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.LimeLight;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import com.pathplanner.lib.auto.AutoBuilder;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private LimeLight frontLimeLight;
  private LimeLight rearLimeLight;
  private LimeLight currentLimeLight;
  private double driveFlip = -1;
  private double angle = 2.0;
  private double joystickDeadzone = 0.5;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    frontLimeLight = m_robotContainer.getFrontLimeLight();
    rearLimeLight = m_robotContainer.getRearLimeLight();
    currentLimeLight = frontLimeLight;
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();  

    //Get controller buttons
    double spinShooter = m_robotContainer.getXboxDriver().getRightTriggerAxis();
    boolean intakeNote = m_robotContainer.getXboxDriver().getRightBumper();
    boolean loadNote = m_robotContainer.getXboxOperator().getLeftBumper();
    double spitNote = m_robotContainer.getXboxOperator().getLeftTriggerAxis();
    boolean trackTarget = false; //m_robotContainer.getXboxDriver().getAButton();
    boolean killArm = m_robotContainer.getXboxDriver().getAButton();
    double dPad = m_robotContainer.getXboxDriver().getPOV();

    boolean isNote = true; //NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("FRC-Note").getBoolean(false);

    // //Adjust current angle of arm based on triggers
    // angle += m_robotContainer.getXboxOperator().getRightTriggerAxis() * 0.5;
    // angle -= m_robotContainer.getXboxOperator().getLeftTriggerAxis() * 0.5;

    if (dPad == 0) {
      angle += 0.125;
    }
    else if (dPad == 180) {
      angle -= 0.125;
    }

    if (angle > 120) {
      angle = 120;
    }
    else if (angle < 2.0) {
      angle = 2.0;
    }

    DriveSubsystem m_drive = m_robotContainer.getDrivetrain();
    double[] pose = {m_drive.getPose().getX(), m_drive.getPose().getY(), m_drive.getPose().getRotation().getDegrees()};

    Arm m_arm = m_robotContainer.getArm();
    Intake m_intake = m_robotContainer.getIntake();
    Shooter m_shooter = m_robotContainer.getShooter();

    if (m_robotContainer.getXboxDriver().getPOV() == 0){
      currentLimeLight = frontLimeLight;
      driveFlip = -1;
    }
    else if (m_robotContainer.getXboxDriver().getPOV() == 180){
      currentLimeLight = rearLimeLight;
      driveFlip = 1;
    }

    currentLimeLight.Update_Limelight_Tracking();

    //Update all of our Shuffleboard data
    SmartDashboard.putNumber("DistanceToTarget", currentLimeLight.getLLTargetDistance());
    SmartDashboard.putNumberArray("RobotPose", pose);
    SmartDashboard.putNumber("DesiredAngle", angle);
    SmartDashboard.putNumber("Current Angle", m_arm.currentAngle());

    //If we push the A Button we attempt to "track" a target with the current limelight (back or front)
    if (trackTarget) {
          //hasValidTarget will return True if we see ANY target that we can identify.  so this would be any apriltag
          if (currentLimeLight.hasValidTarget()) {
            //Here we drive twoard the apriltag.  Not something we will do in a competition but great for Note Tracking
            m_drive.drive(currentLimeLight.getLLDriveY() * driveFlip, currentLimeLight.getLLDriveX() * driveFlip, currentLimeLight.getLLDriveRotation(), false, false);
          }
          else {
            //If we have no valid target we will keep the robot stationary
            m_drive.drive(0.0, 0.0, 0.0, false, false);
          }
    }

    // Runs the intake motors only when a note is not in the intake (intakes a note but stops before loading it into the shooter)
    if (intakeNote) {
      m_intake.intakeNote(isNote);
    }
    else if (loadNote) {
      m_intake.loadNote(isNote); // TODO: Replace this boolean with the proximity sensor data, and write a proper intake function
    }
    else if (spitNote >= joystickDeadzone) {
      m_intake.spitNote(isNote);
    }
    else {
      m_intake.setDesiredVelocity(0.0);
    }

    // Spins the shooters up to the specified speed to fire a note.
    if (spinShooter >= joystickDeadzone) {
      // DLL: Here we will need some maths to determine the velocity based on angle and distance.  It may be better for us to 
      //      create a "shootAmp()" and "shootSpeaker()" functions for the shooter.  It can do the math and angle calculations in the 
      //      subsystem rather than in the robot periodic
      m_shooter.setDesiredVelocity(10);
    }
    else {
      m_shooter.setDesiredVelocity(0.0);
    }

    if (killArm){
      m_arm.killArm();
    }
    else{
      m_arm.moveArm(angle);
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
