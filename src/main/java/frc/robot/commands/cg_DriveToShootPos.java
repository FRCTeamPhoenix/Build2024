package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class cg_DriveToShootPos extends SequentialCommandGroup {
    public cg_DriveToShootPos(DriveSubsystem drive) {
        addCommands(
                drive.generatePath(new Pose2d(15.0, 5.5, new Rotation2d(0.0)))
        );
    }
}
