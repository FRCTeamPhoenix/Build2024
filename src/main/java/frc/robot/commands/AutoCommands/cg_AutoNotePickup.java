package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.cmd_IntakeNote;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OakCamera;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class cg_AutoNotePickup extends ParallelCommandGroup {
    public cg_AutoNotePickup(Intake intake, DriveSubsystem drive) {
        if (OakCamera.findClosestNote() == null) return;
        addCommands(
                new ParallelDeadlineGroup(
                        new cmd_AlignAndDriveToNote(drive),
                        new cmd_IntakeNote(intake)
                ));
    }
}

