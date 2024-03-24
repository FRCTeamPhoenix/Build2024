// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.General;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class cmd_SpinShooter extends Command {
    private final Shooter m_shooter;
    private final DriveSubsystem m_drive;
    private final Arm m_arm;
    private final Intake m_intake;

    public cmd_SpinShooter(Shooter shooter, DriveSubsystem drive, Arm arm, Intake intake) {
        m_shooter = shooter;
        m_drive = drive;
        m_arm = arm;
        m_intake = intake;
        
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (m_drive.getPhotonPose().getX() >= 10.7 || m_intake.getVelocity() <= 0.5 || m_arm.getArmPosition() >= 2){
            m_shooter.setDesiredVelocity(20);
        }
        else {
            m_shooter.killShooter();
        }
        SmartDashboard.putNumber("shooter speed", m_shooter.getVelocity());
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setDesiredVoltage(0);
        if (General.LOGGING)
            System.out.println("End Move Arm Down");
    }

    @Override
    public boolean isFinished() {
        return false;
        // return true;
    }
}
