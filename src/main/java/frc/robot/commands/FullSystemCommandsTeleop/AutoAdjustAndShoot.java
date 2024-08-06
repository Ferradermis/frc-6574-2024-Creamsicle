package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class AutoAdjustAndShoot extends SequentialCommandGroup{
    /** Creates a new AutoAdjustAndShoot */
    public AutoAdjustAndShoot() {
        addCommands(new ParallelCommandGroup(
            new AutoAdjustShooterWrist(RobotContainer.shooterW.limelightGetShooterAngle()),
            //new SetShooterWristPosition(0.188),
            new Shoot()
        ));
    }
}
