package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

//TODO: Not in use atm
public class AutoAdjustAndStartShooter extends Command{
    /** Creates a new AutoAdjustShooterWrist. */
  private double position;
  private double tolerance = 0.005;
  
  public AutoAdjustAndStartShooter(double pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    position = pos;
    addRequirements(RobotContainer.shooterW);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Beginning Auto Adjust");
    //SmartDashboard.putNumber("Shooter start", Timer.getFPGATimestamp());
    RobotContainer.shooter.setShooterVelocityUsingMotionMagic(50);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    position = RobotContainer.shooterW.limelightGetShooterAngle();
    RobotContainer.intake.setIntakeSpeed(0, -0.1, 0);
    RobotContainer.shooterW.setPosition(position);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.wrist.stop();
    SmartDashboard.putNumber("Shooter end", Timer.getFPGATimestamp());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(RobotContainer.shooterW.getPosition() - position) <= tolerance) {
      System.out.println(position);
      System.out.println("Auto Adjust Complete");
      System.out.println(RobotContainer.shooterW.getPosition());
      if (RobotContainer.shooter.getVelocity() >= 50) {
        return true;
      }
      return false;
    }
    else {
      return false;
    }
  }
}
