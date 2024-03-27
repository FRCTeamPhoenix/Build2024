package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class cg_AutoSlowShootNote extends SequentialCommandGroup {
    public cg_AutoSlowShootNote(InterpolatingDoubleTreeMap interpolator, Arm arm, DriveSubsystem drive, Intake intake, Shooter shooter) {
        addCommands(
                new cmd_TargetShooterToSpeaker(interpolator, arm, drive).withTimeout(0.5),
                new cmd_SlowShoot(shooter).withTimeout(1.0),
                new cmd_LoadNote(intake).withTimeout(1.0),
                new cmd_StopIntake(intake),
                new cmd_StopShoot(shooter)
        );
    }
}
