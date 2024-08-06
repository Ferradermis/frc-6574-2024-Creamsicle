package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeNote;

public class ReturnHomeAndIntake extends SequentialCommandGroup{
    public ReturnHomeAndIntake(){
        addCommands(new ParallelCommandGroup(new ReturnToHome(),
        new IntakeNote()));
    }
}
