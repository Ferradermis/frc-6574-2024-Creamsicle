package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SetIntakeSpeeds;
import frc.robot.commands.ShooterWristCommands.SetShooterWristPosition;
import frc.robot.subsystems.ShooterWrist;

//TODO: Update so it doesn't use limelight for now
public class AdjustAndShootShortDistance extends SequentialCommandGroup{
    public AdjustAndShootShortDistance() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new AutoAdjustAndStartShooter(RobotContainer.shooterW.limelightGetShooterAngle()),
            //new SetShooterWristPosition(2.55),
            //new SetShooterWristPosition(2.5),
            new ShootFromShortDistance()
        );
    }
}
