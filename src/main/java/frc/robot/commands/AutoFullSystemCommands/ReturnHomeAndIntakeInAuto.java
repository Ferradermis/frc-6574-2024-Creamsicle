package frc.robot.commands.AutoFullSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShooterWristCommands.SetShooterWristPosition;

public class ReturnHomeAndIntakeInAuto extends SequentialCommandGroup {
    public ReturnHomeAndIntakeInAuto() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new ParallelCommandGroup(
                new SetShooterWristPosition(0.257),
                new IntakeInAuto()
        ).withTimeout(3));
    }
}
