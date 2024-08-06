package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShooterWristCommands.SetShooterWristPosition;

public class AdjustWristAndFeed extends SequentialCommandGroup{
    public AdjustWristAndFeed() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(new ParallelCommandGroup(
            new SetShooterWristPosition(0.208),
            new StartShooter() //205
        ));
    }
}
