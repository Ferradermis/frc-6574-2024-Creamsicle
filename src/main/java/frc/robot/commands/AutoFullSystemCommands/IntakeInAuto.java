// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoFullSystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.RobotContainer;

public class IntakeInAuto extends Command {
  //private double speed;
  /** Creates a new setWristIntakeSpeed. */
  public IntakeInAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
    //this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intake.enableIntakeLimitSwitch();
    RobotContainer.intake.setIntakeSpeed(-RobotConstants.intakeSpeed, -RobotConstants.intakeSpeed, 0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.setShooterVelocityUsingMotionMagic(80);
    RobotContainer.intake.setIntakeSpeed(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.intake.isTriggered())
    {
      return true;
    }
    return false;
  }
}