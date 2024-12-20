// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;



//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**Controller port & Button ID constants.*/
  public static class ControllerConstants {

    public static final int kDriverControllerPort = 0;
  
  }
  /**Config values for Path Planner*/
  public static class PathPlannerConstants {
    /**get the config from GUI while handling thrown exceptions*/
    private static RobotConfig getConfig() {
      try {
        return RobotConfig.fromGUISettings();
      } catch (IOException | ParseException e) {
        throw new RuntimeException(e);
      }
    }
    public static final RobotConfig config = getConfig();
    public static final PIDConstants translationPID = new PIDConstants(7.9, 0.1, 1.25);
    public static final PIDConstants rotationPID = new PIDConstants(7.2,0,1); 
  }

  /**Swerve Module Constants */
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = false;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.08;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15); //15
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = Math.PI*2; // radians
    public static final double kTurningEncoderVelocityFactor = Math.PI*2 / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = Math.PI*2; // radians

    public static final double kDrivingP = 0.055726; //0.055726;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 4;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -0.75;
    public static final double kTurningMaxOutput = 0.75;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 15; // amps
  }
  public static final class NeoMotorConstants {

    public static final double kFreeSpeedRpm = 5676;
  }
  public static final class MotorIDs {
    /*
     *   public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 8;
    public static final int kRearRightDrivingCanId = 6;

    public static final int kFrontLeftTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kFrontRightTurningCanId = 9;
    public static final int kRearRightTurningCanId = 7;

     */
    public static final int frontLeftDriveID = 2;
    public static final int frontLeftTurnID = 3;

    public static final int frontRightDriveID = 8;
    public static final int frontRightTurnID = 9;

    public static final int backLeftDriveID = 4;
    public static final int backLeftTurnID = 5;

    public static final int backRightDriveID = 6;
    public static final int backRightTurnID = 7;
  }
  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(17.6); 
    public static final double kMaxAngularSpeed = 2*Math.PI; // radians per second //2*math.pi

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(20.5); //20.75 old
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(31.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final boolean kGyroReversed = true;
  }
}
