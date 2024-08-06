package frc.robot.commands.AutoFullSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootSubwoofer;
import frc.robot.commands.FullSystemCommandsTeleop.AdjustAndShootSubwoofer;

public class ShootSubwooferInAuto extends SequentialCommandGroup{
    /** Creates a new ShootSubwooferInAuto */
    public ShootSubwooferInAuto() {
        addCommands(
            new ShootSubwoofer().withTimeout(1.25
            ));
    }
}