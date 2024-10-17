package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PhotonNote;
import frc.robot.subsystems.Shooter;

public class cg_AutoIntake extends ParallelRaceGroup {
    public cg_AutoIntake(DriveSubsystem drive, PhotonNote noteSystem, CommandXboxController controller) {
        addCommands(
                //new cmd_AlignBotToNote(drive, noteSystem, controller).withTimeout(5)
                //new cmd_LoadNote(intake)
        );
    }
}

