package frc.robot.subsystems;

import javax.swing.plaf.basic.BasicComboBoxUI.FocusHandler;
import javax.swing.text.StyleContext.SmallAttributeSet;

import com.fasterxml.jackson.databind.deser.std.FromStringDeserializer;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorIDs;
public class DriveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        MotorIDs.frontLeftDriveID,
        MotorIDs.frontLeftTurnID,
        0);
    private final SwerveModule frontRight = new SwerveModule(
        MotorIDs.frontRightDriveID, 
        MotorIDs.frontRightTurnID, 
        0);
    private final SwerveModule backLeft = new SwerveModule(
        MotorIDs.backLeftDriveID, 
        MotorIDs.backLeftTurnID, 
        0);
    private final SwerveModule backRight = new SwerveModule(
        MotorIDs.backRightDriveID,
        MotorIDs.backRightTurnID, 
        0);
    private ADIS16448_IMU gyro = new ADIS16448_IMU();
    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, 
        Rotation2d.fromDegrees(gyro.getGyroAngleZ()), 
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }
    );
    
    public DriveSubsystem() {
        //configure PathPlanner
        AutoBuilder.configure(
            this::getPose, 
            this::resetPose, 
            this::getRelativeSpeeds, 
            (speeds,feedForwards)->driveRelative(speeds), 
            new PPHolonomicDriveController(
                PathPlannerConstants.translationPID, 
                PathPlannerConstants.rotationPID
            ),
            PathPlannerConstants.config, 
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
        );
    }
    @Override
    public void periodic() {
        odometry.update( 
            Rotation2d.fromDegrees(gyro.getGyroAngleZ()), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );
        SmartDashboard.putString("frontLeftPID", frontLeft.getState().angle +" -> "+ frontLeft.getDesiredState().angle);
        SmartDashboard.putString("frontRightPID", frontRight.getState().angle +" -> "+ frontRight.getDesiredState().angle);
        SmartDashboard.putString("backLeftPID", backLeft.getState().angle +" -> "+ backLeft.getDesiredState().angle);
        SmartDashboard.putString("backRightPID", backRight.getState().angle +" -> "+ backRight.getDesiredState().angle);
        SmartDashboard.putData("Front Left",frontLeft);
        SmartDashboard.putData("Front Right",frontRight);
        SmartDashboard.putData("Back Left", backLeft);
        SmartDashboard.putData("Back Right", backRight);
        SmartDashboard.updateValues();
    }
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    public void resetPose(Pose2d pose) {
        odometry.resetPosition( 
            Rotation2d.fromDegrees(gyro.getGyroAngleZ()), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }, pose
        );
    }
    public ChassisSpeeds getRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );
    }
    public void driveRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        frontLeft.setDesiredState(states[1]);
        frontRight.setDesiredState(states[0]);
        backLeft.setDesiredState(states[3]);
        backRight.setDesiredState(states[2]);
    }
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        rot *= DriveConstants.kMaxAngularSpeed;
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed,ySpeed,rot);
        //new ChassisSpeeds(xSpeed,ySpeed,rot).toFieldRelativeSpeeds(Rotation2d.fromDegrees(getHeading()));
        if (fieldRelative) speeds.toFieldRelativeSpeeds(Rotation2d.fromDegrees(getHeading()));
        driveRelative(speeds);
    }
    public double getHeading() {
        return gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
}
