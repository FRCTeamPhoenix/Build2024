package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OakCamera;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PhotonClass;
import frc.utils.FireControlUtil;

public class cg_FetchNoteAndShoot extends SequentialCommandGroup {
    public cg_FetchNoteAndShoot(Intake intake, Shooter shooter, DriveSubsystem driveSubsystem,Arm arm, PhotonClass photonClass, OakCamera oakCamera, FireControlUtil util, CommandXboxController controller) {
        addCommands(
                new cmd_DriveNearestNote(oakCamera,driveSubsystem),
                new cg_FloorIntake(intake, arm),
                new cg_ShootAndMoveArm(util, arm, driveSubsystem, photonClass, shooter, intake, controller)
        );
    }
}
