// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLight;

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
  private String currentLimeLightString = "Front";
  private double driveFlip = -1;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    frontLimeLight = m_robotContainer.getm_frontLimeLight();
    rearLimeLight = m_robotContainer.getm_rearLimeLight();
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
    frontLimeLight = m_robotContainer.getm_frontLimeLight();
    rearLimeLight = m_robotContainer.getm_rearLimeLight();
    DriveSubsystem m_drive = m_robotContainer.getm_driveTrain();
    double[] pose = {m_drive.getPose().getX(), m_drive.getPose().getY(), m_drive.getPose().getRotation().getDegrees()};

    if (m_robotContainer.getxboxDriver().getPOV() == 0){
      currentLimeLight = frontLimeLight;
      currentLimeLightString = "Front";
      driveFlip = -1;
    }
    else if (m_robotContainer.getxboxDriver().getPOV() == 180){
      currentLimeLight = rearLimeLight;
      currentLimeLightString = "Rear";
      driveFlip = 1;
    }
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    currentLimeLight.Update_Limelight_Tracking();
    SmartDashboard.putNumber("Steer: ", currentLimeLight.getLLDriveRotation());
    SmartDashboard.putNumber("DriveX: ", currentLimeLight.getLLDriveX());
    SmartDashboard.putNumber("DriveY: ", currentLimeLight.getLLDriveY());
    SmartDashboard.putNumber("TA: ", currentLimeLight.getLLTargetArea());
    SmartDashboard.putNumber("Distance to target: ", currentLimeLight.getLLTargetDistance());
    SmartDashboard.putString("Current LimeLight: ", currentLimeLightString);
    SmartDashboard.putNumberArray("RobotPose", pose);
    CommandScheduler.getInstance().run();
    currentLimeLight.Update_Limelight_Tracking();
    boolean trackTarget = m_robotContainer.getxboxDriver().getAButton();
    if (trackTarget)
        {
          if (currentLimeLight.hasValidTarget())
          {
            m_drive.drive(currentLimeLight.getLLDriveY() * driveFlip, currentLimeLight.getLLDriveX() * driveFlip, currentLimeLight.getLLDriveRotation(), false, false);
          }
          else
          {
                m_drive.drive(0.0, 0.0, 0.0, false, false);
          }
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
