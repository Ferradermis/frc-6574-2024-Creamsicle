// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

//TODO: Double-check to make sure relative encoder stuff works
public class ShooterWrist extends SubsystemBase {

  public CANSparkMax shooterWristMotor;

  private SparkPIDController shooterWristPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  //private double maxSpeed = 0.25;
  //private double deadBand = 0.1;

  /** Creates a new ShooterWrist. */
  public ShooterWrist() {
    shooterWristMotor = new CANSparkMax(Constants.RobotConstants.shooterWristCANID, MotorType.kBrushless);
    shooterWristMotor.restoreFactoryDefaults();

    shooterWristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    shooterWristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    shooterWristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    shooterWristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    shooterWristPIDController = shooterWristMotor.getPIDController();


    //m_AbsoluteEncoder.setPositionConversionFactor(360);
    //m_AbsoluteEncoder.setVelocityConversionFactor(1);
    shooterWristMotor.setInverted(false);
    shooterWristMotor.setIdleMode(IdleMode.kBrake);

    shooterWristMotor.setSmartCurrentLimit(45);

    kP = 7; // 7
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.7;
    kMaxOutput = .5;
    kMinOutput = -.5;

    shooterWristPIDController.setP(kP);
    shooterWristPIDController.setI(kI);
    shooterWristPIDController.setD(kD);
    shooterWristPIDController.setIZone(kIz);
    shooterWristPIDController.setFF(kFF);
    shooterWristPIDController.setOutputRange(kMinOutput, kMaxOutput);

    shooterWristPIDController.setPositionPIDWrappingEnabled(true);
    shooterWristPIDController.setPositionPIDWrappingMinInput(0);
    shooterWristPIDController.setPositionPIDWrappingMaxInput(1);
  }

  @Override

  public void periodic() {
    // double position = limelightGetShooterAngle();
    // SmartDashboard.putNumber("limelight shooter", position);
    SmartDashboard.putNumber("Wrist Encoder", getPosition());
  }

  public void setSpeed(double speed)
  {
    shooterWristMotor.set(speed);
  }

  public void stop()
  {
    shooterWristMotor.stopMotor();
  }

  public void setPosition(double position) {
    shooterWristPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public double getPosition() {
    return shooterWristMotor.getEncoder().getPosition();
  }

  // Method to adjust our shooter wrist angle based on distance from speaker
  // Returns the encoder value to set the position of the wrist
  // //TODO: Redo this equation
  // public double limelightGetShooterAngle() {
  //   double distance = RobotContainer.limelight.getDistanceToTarget();
  //   return (6.83518/(distance + 42.3178)) + 0.161171;
  // }
}

