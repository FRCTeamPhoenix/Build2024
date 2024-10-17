// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OakCamera;
import frc.utils.NotePoseGenerator;
import frc.utils.OakCameraObject;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private final Field2d field = new Field2d();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        DataLogManager.start();
        //Startup the Camera Server for the driver
        CameraServer.startAutomaticCapture(0);

        m_robotContainer.initPose();
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

        if (m_robotContainer.getIntake().getVelocity() >= 0.5){
            m_robotContainer.getXboxOperator().getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        }
        else{
            m_robotContainer.getXboxOperator().getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
        }

        m_robotContainer.updatePose();

        SmartDashboard.putBoolean("Intake Running", m_robotContainer.getIntake().getVelocity() != 0.0);
        DriveSubsystem m_drive = m_robotContainer.getDrivetrain();
        Arm m_arm = m_robotContainer.getArm();

        double[] pose = {m_drive.getPhotonPose().getX(), m_drive.getPhotonPose().getY(), m_drive.getPhotonPose().getRotation().getRadians()};
        SmartDashboard.putNumberArray("PhotonPose", pose);

        double[] odometryPose = {m_drive.getPose().getX(), m_drive.getPose().getY(), m_drive.getPose().getRotation().getRadians()};
        SmartDashboard.putNumberArray("OdometryPose", odometryPose);

        SmartDashboard.putNumber("Current Angle", m_arm.getArmPosition());

        boolean hasNote = SmartDashboard.getString("FRC-Note", "Not Found").equals("Found");
        SmartDashboard.putBoolean("Note in Intake?", hasNote);

        Pose2d speakerPose = Constants.VisionConstants.kTagLayout.getTagPose(4).get().toPose2d();
        Transform2d transformToSpeaker = m_robotContainer.currentPose2d.minus(speakerPose);
        double distance = Math.sqrt(Math.pow(transformToSpeaker.getX(), 2) + Math.pow(transformToSpeaker.getY(), 2));
        SmartDashboard.putNumber("Distance To Speaker", distance);
        
        SmartDashboard.putNumber("Total Memory", Runtime.getRuntime().totalMemory() / 1024);
        SmartDashboard.putNumber("Memory Used", (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1024);
        SmartDashboard.putNumber("Memory Available", Runtime.getRuntime().freeMemory() / 1024);

        SmartDashboard.putNumber("shooter speed", m_robotContainer.getShooter().getVelocity());
        SmartDashboard.putNumber("note yaw", m_robotContainer.note_sys.getNearestNoteYaw());

        boolean status = !m_robotContainer.getIntake().sensorStatus();
        SmartDashboard.putBoolean("note status", status);
        
       // SmartDashboard.putNumber("Climber", m_robotContainer.getClimber().encoderPosition());
        //SmartDashboard.putBoolean("Can Climb", m_robotContainer.getClimber().encoderPosition() > -60);
        //SmartDashboard.putBoolean("Note Visible", OakCamera.hasValidTarget());

        //field.setRobotPose(NotePoseGenerator.generateNotePose(OakCamera.findClosestNote(), m_drive.getPhotonPose()));
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
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

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

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

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
