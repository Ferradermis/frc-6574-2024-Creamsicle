package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

//I am very scared to run this. - Ace
public class LimelightDriveToTarget extends Command{
    private double rotation;
    private double forward;
    private boolean fieldRelative;
    private double yaw = 0;

    /** Creates a new LimelightDriveToTarget */
    public LimelightDriveToTarget() {
        addRequirements(RobotContainer.m_robotDrive);
    }
    public void initialize() {
        rotation = RobotContainer.limelight.limelight_aim_proportional();
        forward = RobotContainer.limelight.limelight_range_proportional();
        fieldRelative = false;
    }

    public void execute() {
        RobotContainer.m_robotDrive.drive(
        -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
        -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband), 
        rotation, 
        fieldRelative, 
        true);
    }

    public void end(boolean interrupted) {
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        RobotContainer.m_robotDrive.drive(
                -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(RobotContainer.m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true);
            
    }

    @Override
    public boolean isFinished() {
        if ((LimelightHelpers.getTX("limelight") >= -10 && LimelightHelpers.getTX("limelight") <= -4) //&& (Math.abs(LimelightHelpers.getTY("limelight")) <= 0.25)
        ) {
            yaw = RobotContainer.m_robotDrive.getGyroYaw();
            RobotContainer.m_robotDrive.drive(
                -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(RobotContainer.m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true);
            RobotContainer.m_robotDrive.setGyroYaw(yaw);
            return true;
        }
        else if (LimelightHelpers.getTV("limelight") == false)
        {
            yaw = RobotContainer.m_robotDrive.getGyroYaw();
            RobotContainer.m_robotDrive.drive(
                -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(RobotContainer.m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true);
            RobotContainer.m_robotDrive.setGyroYaw(yaw);
            return true;
        } 
        else 
        {
            return false;
        }
    }
}
