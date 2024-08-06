// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;

// Limelight subsystem taken from team 4400 Cerbotics - looks to be updated from last time I checked their code base
public class VisionSubsystem {
  private double ty;
    private final DriveSubsystem m_drive;

    private final SwerveDrivePoseEstimator m_poseEstimator;

    Alliance alliance = Alliance.Blue;

    Field2d m_field = new Field2d();

    Debouncer poseDebouncer = new Debouncer(0.1, DebounceType.kRising);

    public VisionSubsystem(DriveSubsystem m_drive){
        this.m_drive = m_drive;

        m_poseEstimator = 
            new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            m_drive.getRotation2d(), m_drive.getModulePositions(), 
            new Pose2d(0,0 , m_drive.getRotation2d()));

        SmartDashboard.putData("Field", m_field);

        vision_thread();
    }

    //Start the vision systeam in a different CPU thread for better command scheduler performance
    public void vision_thread(){
        try{
            new Thread(() -> {
                while(true){
                    periodic();
                    try {
                        Thread.sleep(20);
                    } catch (InterruptedException e){
                     //Auto-generated catch block
                     e.printStackTrace();
                    }
                }
            }).start();
        } catch(Exception e){}
    }

    public void periodic(){

      SmartDashboard.putNumber("Limelight TX", LimelightHelpers.getTX("limelight"));
      SmartDashboard.putNumber("Limelight TY", LimelightHelpers.getTY("limelight"));
      SmartDashboard.putNumber("limelight distance", getDistanceToTarget());
      ty = LimelightHelpers.getTY("limelight");
      if (LimelightHelpers.getTV("limelight")) {
        SmartDashboard.putString("Has Valid Target", "True");
      }
      else {
        SmartDashboard.putString("Has Valid Target", "False");
      }
      /*
        odometryWvision();
        //setDynamicVisionStdDevs();

        SmartDashboard.putString("Alliance", alliance.toString());

        SmartDashboard.putNumber("Num of tags", getNumofDetectedTargets());
        */
    }

    /*
    public void setAlliance(Alliance alliance){
        this.alliance = alliance;
    }

    public void resetPoseEstimator(Pose2d pose){
        m_poseEstimator.resetPosition(m_drive.getRotation2d(), m_drive.getModulePositions(), pose);
      }
    */ 
    /*
    public Pose2d estimatedPose2d(){
        return m_poseEstimator.getEstimatedPosition();
    }
    */
    /* 
    public Translation2d getEstimationTranslation(){
        return m_poseEstimator.getEstimatedPosition().getTranslation();
    }
    
    public Rotation2d getEstimationRotation(){
        return m_poseEstimator.getEstimatedPosition().getRotation();
    }
    
    public double getEstimationAngle(){
        return m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    }

  public void odometryWvision(){
    m_poseEstimator.update(m_drive.getRotation2d(), m_drive.getModulePositions());

    LimelightHelpers.Results results = 
        LimelightHelpers.getLatestResults(VisionConstants.tagLimelightName).targetingResults;

    if(LimelightHelpers.getTV(VisionConstants.tagLimelightName)){
      Pose2d camPose = LimelightHelpers.toPose2D(results.botpose_wpiblue);
      m_poseEstimator.addVisionMeasurement(camPose, 
      Timer.getFPGATimestamp() - (results.latency_capture / 1000.0)
       - (results.latency_pipeline / 1000.0));
      m_field.getObject("Cam est Pose").setPose(camPose);
    } else {
      m_field.getObject("Cam est Pose").setPose(m_poseEstimator.getEstimatedPosition());
    }

    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
  }

  public void setDynamicVisionStdDevs(){
    int numDetectedTargets = getNumofDetectedTargets();
    double stdsDevXY = 0.0;
    double stdsDevDeg = 0.0;

    if(numDetectedTargets <= 2){
      stdsDevXY = 0.1;
      stdsDevDeg = 0.1;
    } else {
      stdsDevXY = Math.abs(m_drive.getAverageDriveSpeed());
      stdsDevDeg = Math.abs(m_drive.getAngularAcceleration()) / 200;
    }

    Matrix<N3, N1> visionMat = MatBuilder.fill(Nat.N3(), Nat.N1(), stdsDevXY, stdsDevXY, stdsDevDeg);

    m_poseEstimator.setVisionMeasurementStdDevs(visionMat);
  }

  public int getNumofDetectedTargets(){
    return LimelightHelpers
    .getLatestResults(VisionConstants.tagLimelightName).targetingResults.targets_Fiducials.length;
  }

  */

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  public double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .0012;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= DriveConstants.kMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

   // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  public double getDistanceToTarget() {
    double targetOffsetAngle_Vertical = ty;

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 24.5; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 24.875; 

    // distance from the target to the floor
    double goalHeightInches = 51.875; 

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    return distanceFromLimelightToGoalInches;
  }
}
