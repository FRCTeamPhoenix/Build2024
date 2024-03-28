package frc.robot.commands.AutoCommands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.PhotonClass;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.Arm;
public class cg_AutoIntakeToFloor extends ParallelCommandGroup {
    public cg_AutoIntakeToFloor(Intake intake, Arm arm) {
        addCommands(
                new ParallelCommandGroup(
                        new cmd_MoveArmToPosition(0.05, 0.25, arm).withTimeout(1)
                )
        );
    }
}

