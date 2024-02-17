package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class cg_ShootNote extends ParallelCommandGroup{
    public cg_ShootNote(Intake intake, Shooter shooter){
        addCommands(
            new cmd_Shoot(shooter),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new cmd_LoadNote(intake)
                )
            
        );
    }
}
