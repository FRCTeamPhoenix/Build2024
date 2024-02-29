package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.cmd_TargetShooterToSpeaker;
import frc.robot.subsystems.PhotonClass;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.Arm;
public class cg_ShootAndMoveArm extends ParallelCommandGroup {
    public cg_ShootAndMoveArm(Intake intake, Shooter shooter,Arm arm,InterpolatingDoubleTreeMap interpolator,PhotonClass photonCam,boolean isAllianceRed ) {
        addCommands(
                
                new SequentialCommandGroup(
                        new WaitCommand(1.0),
                        new cmd_LoadNote(intake),
                        new cmd_TargetShooterToSpeaker(interpolator, photonCam, arm, isAllianceRed)
                ),
                new cmd_Shoot(shooter)
        );
    }
}

