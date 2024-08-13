// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  public TalonFX kShooterTopLeft;
  public TalonFX kShooterBottomLeft;
  public TalonFX kShooterTopRight;
  public TalonFX kShooterBottomRight;

  // public TalonFXConfiguration shooterAngleFxConfiguration = new TalonFXConfiguration();
  public TalonFXConfiguration shooterVelocityFxConfigurationTopL = new TalonFXConfiguration();
  public TalonFXConfiguration shooterVelocityFxConfigurationBottomL = new TalonFXConfiguration();
  public TalonFXConfiguration shooterVelocityFxConfigurationTopR = new TalonFXConfiguration();
  public TalonFXConfiguration shooterVelocityFxConfigurationBottomR = new TalonFXConfiguration();
  public CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
  double mvarShooterSpinupStartTime = 0;

    /* Start at velocity 0, no feed forward, use slot 1 */ //maybe disregard the previous comment, i don't remember what it means
  private final VelocityDutyCycle m_torqueVelocity = new VelocityDutyCycle(
    0, 
    0, 
    false, 0, 
    1, 
    false, false, false
    );

  private final MotionMagicVelocityVoltage m_motionMagicVelocity = new MotionMagicVelocityVoltage(
    0, 
    800, 
    false, 0, 
    0, false, 
    false, false
    );
    
  /** Creates a new Shooter. */
  public Shooter() {
    //configuration settings, slot 1 might get booted?
    /* 
    shooterVelocityFxConfiguration.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
    shooterVelocityFxConfiguration.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    shooterVelocityFxConfiguration.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
    */

    // TODO: Needs tuning when I get the robot back
    shooterVelocityFxConfigurationTopL.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
    shooterVelocityFxConfigurationTopL.Slot0.kV = 0.14; //0.125 // A velocity target of 1 rps results in 0.12 V output
    shooterVelocityFxConfigurationTopL.Slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    shooterVelocityFxConfigurationTopL.Slot0.kP = 0.44; //0.44 // An error of 1 rps results in 0.11 V output
    shooterVelocityFxConfigurationTopL.Slot0.kI = 0; // no output for integrated error
    shooterVelocityFxConfigurationTopL.Slot0.kD = 0; //0.1 // no output for error derivative

    shooterVelocityFxConfigurationBottomL.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
    shooterVelocityFxConfigurationBottomL.Slot0.kV = 0.14; //0.125 // A velocity target of 1 rps results in 0.12 V output
    shooterVelocityFxConfigurationBottomL.Slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    shooterVelocityFxConfigurationBottomL.Slot0.kP = 0.44; //0.44 // An error of 1 rps results in 0.11 V output
    shooterVelocityFxConfigurationBottomL.Slot0.kI = 0; // no output for integrated error
    shooterVelocityFxConfigurationBottomL.Slot0.kD = 0; //0.1 // no output for error derivative

    shooterVelocityFxConfigurationTopR.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
    shooterVelocityFxConfigurationTopR.Slot0.kV = 0.14; //0.132 // A velocity target of 1 rps results in 0.12 V output
    shooterVelocityFxConfigurationTopR.Slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    shooterVelocityFxConfigurationTopR.Slot0.kP = 0.44; //0.88 // An error of 1 rps results in 0.11 V output
    shooterVelocityFxConfigurationTopR.Slot0.kI = 0; // no output for integrated error
    shooterVelocityFxConfigurationTopR.Slot0.kD = 0; // no output for error derivative

    shooterVelocityFxConfigurationBottomR.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
    shooterVelocityFxConfigurationBottomR.Slot0.kV = 0.14; //0.132 // A velocity target of 1 rps results in 0.12 V output
    shooterVelocityFxConfigurationBottomR.Slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    shooterVelocityFxConfigurationBottomR.Slot0.kP = 0.44; //0.88 // An error of 1 rps results in 0.11 V output
    shooterVelocityFxConfigurationBottomR.Slot0.kI = 0; // no output for integrated error
    shooterVelocityFxConfigurationBottomR.Slot0.kD = 0; // no output for error derivative

    var motionMagicConfigsTopL = shooterVelocityFxConfigurationTopL.MotionMagic;
    var motionMagicConfigsBottomL = shooterVelocityFxConfigurationBottomL.MotionMagic;
    var motionMagicConfigsTopR = shooterVelocityFxConfigurationTopR.MotionMagic;
    var motionMagicConfigsBottomR = shooterVelocityFxConfigurationBottomR.MotionMagic;
    motionMagicConfigsTopL.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigsTopL.MotionMagicJerk = 0; //16000 
    motionMagicConfigsBottomL.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigsBottomL.MotionMagicJerk = 0; //16000 
    motionMagicConfigsTopR.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigsTopR.MotionMagicJerk = 0; 
    motionMagicConfigsBottomR.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigsBottomR.MotionMagicJerk = 0; 
    
    //actually the motors
    kShooterTopLeft = new TalonFX(Constants.RobotConstants.shooterTopLeftCANID);
    kShooterTopLeft.getConfigurator().apply(shooterVelocityFxConfigurationTopL);
    kShooterTopLeft.setNeutralMode(NeutralModeValue.Coast);
    kShooterTopLeft.getConfigurator().apply(currentLimitConfig.withStatorCurrentLimit(90));
    kShooterTopLeft.getConfigurator().apply(currentLimitConfig.withSupplyCurrentLimit(50));

    kShooterBottomLeft = new TalonFX(Constants.RobotConstants.shooterBottomLeftCANID);
    kShooterBottomLeft.getConfigurator().apply(shooterVelocityFxConfigurationBottomL);
    kShooterBottomLeft.setNeutralMode(NeutralModeValue.Coast);
    kShooterBottomLeft.getConfigurator().apply(currentLimitConfig.withStatorCurrentLimit(90));
    kShooterBottomLeft.getConfigurator().apply(currentLimitConfig.withSupplyCurrentLimit(50));

    kShooterTopRight = new TalonFX(Constants.RobotConstants.shooterTopRightCANID);
    kShooterTopRight.getConfigurator().apply(shooterVelocityFxConfigurationTopR);
    kShooterTopRight.setNeutralMode(NeutralModeValue.Coast);
    kShooterTopRight.getConfigurator().apply(currentLimitConfig.withStatorCurrentLimit(90));
    kShooterTopRight.getConfigurator().apply(currentLimitConfig.withSupplyCurrentLimit(50));
    
    kShooterBottomRight = new TalonFX(Constants.RobotConstants.shooterBottomRightCANID);
    kShooterBottomRight.getConfigurator().apply(shooterVelocityFxConfigurationBottomR);
    kShooterBottomRight.setNeutralMode(NeutralModeValue.Coast);
    kShooterBottomRight.getConfigurator().apply(currentLimitConfig.withStatorCurrentLimit(90));
    kShooterBottomRight.getConfigurator().apply(currentLimitConfig.withSupplyCurrentLimit(50));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(double speed) {
    mvarShooterSpinupStartTime=Timer.getFPGATimestamp();
    kShooterTopLeft.set(-speed);
    kShooterBottomLeft.set(-speed);
    kShooterTopRight.set(-speed);
    kShooterBottomRight.set(-speed);
  }

  //TODO: Rework methods
  public double getVelocity(){
    return kShooterTopLeft.getVelocity().getValue();
  }
  
  public void setShooterVelocity(double desiredRotationsPerSecond) {
    kShooterTopLeft.setControl(m_torqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(1));
    kShooterBottomLeft.setControl(m_torqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(1));
    kShooterTopRight.setControl(m_torqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(1));
    kShooterBottomRight.setControl(m_torqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(1));
  }

  public void setShooterVelocityUsingMotionMagic(double desiredVelocityMotionMagic) {
    kShooterTopLeft.setControl(m_motionMagicVelocity.withVelocity(desiredVelocityMotionMagic));
    kShooterBottomLeft.setControl(m_motionMagicVelocity.withVelocity(desiredVelocityMotionMagic));
    kShooterTopRight.setControl(m_motionMagicVelocity.withVelocity(desiredVelocityMotionMagic));
    kShooterBottomRight.setControl(m_motionMagicVelocity.withVelocity(desiredVelocityMotionMagic));
  }
}
