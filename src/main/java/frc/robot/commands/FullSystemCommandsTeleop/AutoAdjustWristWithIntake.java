package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class AutoAdjustWristWithIntake extends SequentialCommandGroup{
    public AutoAdjustWristWithIntake() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new AutoAdjustShooterWrist(RobotContainer.shooterW.limelightGetShooterAngle()).withTimeout(1));
    }
    
}
