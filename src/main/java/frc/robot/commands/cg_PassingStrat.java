package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class cg_PassingStrat extends SequentialCommandGroup {
    public cg_PassingStrat(Arm arm, DriveSubsystem drive, Shooter shooter, Intake intake, CommandXboxController controller, int desiredHeading) {
        addCommands(
                new cmd_MoveArmToPosition(0.392, 0.3, arm).withTimeout(0.5),
                new cg_PassNote(intake, shooter)
        );
    }
}