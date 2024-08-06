package frc.robot.commands.AutoFullSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FullSystemCommandsTeleop.AdjustAndShootShortDistance;

public class ShootNoteInAuto extends SequentialCommandGroup{
    /** Creates a new ShootNoteInAuto */
    public ShootNoteInAuto() {
        addCommands(
            new AdjustAndShootShortDistance().withTimeout(0.75)
        );
    }
}
