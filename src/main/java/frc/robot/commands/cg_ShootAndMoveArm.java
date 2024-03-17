package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PhotonClass;
import frc.robot.subsystems.Shooter;
import frc.utils.FireControlUtil;

public class cg_ShootAndMoveArm extends SequentialCommandGroup {
    public cg_ShootAndMoveArm(FireControlUtil util, Arm arm, DriveSubsystem drive, PhotonClass camera, Shooter shooter, Intake intake, CommandXboxController controller) {
        addCommands(
                new cmd_TargetShooterToSpeaker(util, arm, drive).withTimeout(1),
                new cmd_AlignShooterToSpeaker(drive, camera, controller).withTimeout(1),
                new cg_ShootNote(intake, shooter).withTimeout(2)
        );
    }
}