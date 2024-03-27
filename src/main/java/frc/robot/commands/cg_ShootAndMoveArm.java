package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PhotonClass;
import frc.robot.subsystems.Shooter;

public class cg_ShootAndMoveArm extends SequentialCommandGroup {
    public cg_ShootAndMoveArm(InterpolatingDoubleTreeMap interpolator, Arm arm, DriveSubsystem drive, PhotonClass camera, Shooter shooter, Intake intake, CommandXboxController controller) {
        addCommands(
            //Need to probably take the timeout from here as that MAY be why we shot into the stands.   we couldnt align in 1 second.
                new cmd_TargetShooterToSpeaker(interpolator, arm, drive).withTimeout(1),
                new cmd_AlignShooterToSpeaker(drive, camera, controller).withTimeout(1),
                new cg_ShootNote(intake, shooter).withTimeout(2)
        );
    }
}