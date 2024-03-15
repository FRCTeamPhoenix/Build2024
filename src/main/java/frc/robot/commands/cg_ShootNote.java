package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class cg_ShootNote extends SequentialCommandGroup {
    public cg_ShootNote(Intake intake, Shooter shooter) {
        addCommands(
                new cmd_Shoot(shooter).withTimeout(1),
                new cmd_LoadNote(intake)
        );
    }
}

