package frc.robot.commands.AutoFullSystemCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class StopShooter extends SequentialCommandGroup{
    /** Creates a new StopShooter */
    public StopShooter() {
        addCommands(
            new RunCommand(() -> RobotContainer.shooter.setShooterSpeed(0), RobotContainer.shooter).withTimeout(5));
        ;
    }
}
