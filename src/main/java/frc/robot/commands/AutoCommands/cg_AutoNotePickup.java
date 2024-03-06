package frc.robot.commands.AutoCommands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OakCamera;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.PhotonClass;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
public class cg_AutoNotePickup extends ParallelCommandGroup {
    public cg_AutoNotePickup(Intake intake, Arm arm, OakCamera oakCamera, DriveSubsystem drive) {
        addCommands(
                new ParallelDeadlineGroup(
                        new cmd_IntakeNote(intake),
                        new cmd_AutoDriveNearestNote(oakCamera, drive)
                )
        );
    }
}

