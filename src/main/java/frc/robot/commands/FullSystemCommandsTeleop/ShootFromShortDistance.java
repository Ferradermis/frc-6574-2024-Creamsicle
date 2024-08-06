package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.RobotContainer;

public class ShootFromShortDistance extends Command{
    //private double speed;
  /** Creates a new setWristIntakeSpeed. */
  public ShootFromShortDistance() {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(RobotContainer.intake);
    addRequirements(RobotContainer.shooter);
    //this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Shooter start", Timer.getFPGATimestamp());
    RobotContainer.shooter.setShooterVelocityUsingMotionMagic(50);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Shooter end", Timer.getFPGATimestamp());
    if (RobotContainer.shooter.getVelocity() >= 50){
      RobotContainer.intake.disableIntakeLimitSwitch();
      RobotContainer.intake.setIntakeSpeed(0, -1, 1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.setShooterSpeed(0);
    RobotContainer.intake.setIntakeSpeed(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
