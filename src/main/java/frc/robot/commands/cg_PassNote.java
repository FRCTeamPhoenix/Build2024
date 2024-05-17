package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class cg_PassNote extends SequentialCommandGroup {
    public cg_PassNote(Intake intake, Shooter shooter) {
        addCommands(
                new cmd_Pass(shooter).withTimeout(1),
                new cmd_LoadNote(intake)
        );
    }
}

