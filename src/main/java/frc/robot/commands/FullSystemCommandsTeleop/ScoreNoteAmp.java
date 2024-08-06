package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.SetIntakeSpeeds;
import frc.robot.commands.ShooterWristCommands.SetShooterWristPosition;

public class ScoreNoteAmp extends SequentialCommandGroup {
  /** Creates a new ScoreNoteAmp. */
  public ScoreNoteAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
                               new SetIntakeSpeeds(0, -0.2, 0).withTimeout(0.5),
                               new SetShooterWristPosition(RobotConstants.shooterWristAmpPos))

    );
  }
}
