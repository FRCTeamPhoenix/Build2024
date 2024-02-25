package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class cg_StopShootNote extends ParallelCommandGroup {
    public cg_StopShootNote(Intake intake, Shooter shooter) {
        addCommands(
                new cmd_StopIntake(intake),
                new cmd_StopShoot(shooter)
        );
    }
}
