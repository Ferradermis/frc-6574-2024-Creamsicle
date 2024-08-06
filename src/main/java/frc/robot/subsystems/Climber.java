/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Climber extends SubsystemBase {

  public CANSparkMax leftMotor;
  public CANSparkMax rightMotor;
  private SparkLimitSwitch climberReverseLimit;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private double maxSpeed = 0.4;
  //private float maxElevatorExtension = 32.5f;
  
  /**Creates a new Climber. 
   *rightMotor follows leftMotor and is inverted.
   */
  public Climber() {
    leftMotor = new CANSparkMax(RobotConstants.climberLeftCANID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(RobotConstants.climberRightCANID, MotorType.kBrushless);

    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    climberReverseLimit = leftMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftMotor.setSmartCurrentLimit(35);
    rightMotor.setSmartCurrentLimit(35);

    leftMotor.setInverted(false);

    rightMotor.follow(leftMotor, true);
  }

  /** Drives the Climber motors at a set speed. 
  * Because the right motor follows the left, we don't need them to be called seperately.
  */
  public void driveClimber(double speed) {
    leftMotor.set(speed * maxSpeed);
  }

  @Override
  public void periodic() {
    climberReverseLimit.enableLimitSwitch(true);
    }
  }
  
