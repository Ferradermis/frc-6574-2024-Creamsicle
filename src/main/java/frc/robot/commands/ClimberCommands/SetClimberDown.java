package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetClimberDown extends Command{
    /** Creates a new SetClimberDown */
    public SetClimberDown(){
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.climber);

    }

    // Called when the command is initially scheduled.
    public void initialize() {
        RobotContainer.climber.driveClimber(1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      RobotContainer.climber.driveClimber(0);
    }
}
