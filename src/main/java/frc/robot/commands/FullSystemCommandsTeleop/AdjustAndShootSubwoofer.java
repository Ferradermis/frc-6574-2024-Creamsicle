package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootSubwoofer;
import frc.robot.commands.ShooterWristCommands.SetShooterWristPosition;

public class AdjustAndShootSubwoofer extends SequentialCommandGroup{
    public AdjustAndShootSubwoofer()
    {
        addCommands(new ParallelCommandGroup(
            new SetShooterWristPosition(0.238),
            //new AutoAdjustShooterWrist(RobotContainer.limelight.getDistanceToTarget()),
            new ShootSubwoofer()));
    }
}
