package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class StartShooter extends Command{
    //private double speed;
  /** Creates a new setWristIntakeSpeed. */
  public StartShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
    addRequirements(RobotContainer.shooter);
    //this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intake.disableIntakeLimitSwitch();
    SmartDashboard.putNumber("Shooter start", Timer.getFPGATimestamp());
    RobotContainer.shooter.setShooterVelocityUsingMotionMagic(80);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("Shooter end", Timer.getFPGATimestamp());
    RobotContainer.shooter.setShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
