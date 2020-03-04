package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * All variables tied to the physical behavior of the robot. Tuning, reverse booleans, etc.
 */
public class Vars {
  
  // robot characteristics
  public static final double WHEEL_DIAMETER = 6; // inches
  public static final double MAX_VELOCITY = 114; // inches/sec
  public static final double MAX_ACCELERATION = 220; // inches/sec^2
  public static final StatorCurrentLimitConfiguration DRIVETRAIN_CURRENTLIMIT = // Current limiting applied to the drivetrain
    new StatorCurrentLimitConfiguration(
      // limiting?, limit (A), threshold (A), threshold time (s)
      true, 60, 70, 0.5
    );
  
  // flipped motors
  public static final boolean LEFT_DRIVE_INVERTED = false;
  public static final boolean RIGHT_DRIVE_INVERTED = false;
  public static final boolean TURRET_REVERSED = false;
  public static final boolean SHOOTER_LEFT_REVERSED = true;
  public static final boolean SHOOTER_RIGHT_REVERSED = false;
  public static final boolean HOPPER_WALL_REVERSED = true;
  public static final boolean HOPPER_FLOOR_REVERSED = false;
  public static final boolean FEED_REVERSED = true;
  public static final boolean ARM_REVERSED = false;
  public static final boolean INTAKE_REVERSED = true;
  
  // flipped encoders
  public static final boolean TURRET_ENCODER_REVERSED = false;
  public static final boolean SHOOTER_ENCODER_REVERSED = true;
  public static final boolean ARM_ENCODER_REVERSED = false;
  
  // turret
  public static final double MAX_ROTATION = 60; // degrees clockwise
  public static final double MIN_ROTATION = -240; // degrees clockwise
  public static final double TURRET_TOLERANCE = 2; // degrees
  public static final double TURRET_KP = 10; // TODO further tune this value
  
  // shooter
  public static final double SHOOTER_DEFAULT = 5500; // rpm
  public static final double SHOOTER_TOLERANCE = 100; // +rpm
  // public static final double SHOOTER_PRETIME = 0.2; // seconds (for hopper and feed to pulse back)
  public static final double SHOOTER_PRE = -0.2; // percent (for hopper and feed to pulse back)
  public static final double SHOOTER_KP = 0.15;
  
  // intake
  public static final double FEED_PERCENT = 1.0;
  public static final double FEED_PERCENT_BACK = -1.0;

  // hopper
  public static final double HOPPER_WALL_PERCENT = 1.0; // 0.5
  public static final double HOPPER_FLOOR_PERCENT = 1.0;
  public static final double HOPPER_WALL_PERCENT_BACK = -1.0;
  public static final double HOPPER_FLOOR_PERCENT_BACK = -1.0;
  
  // arm
  public static final double ARM_IN = 0; // degrees
  public static final double ARM_PLAYER = 83; // degrees
  public static final double ARM_OUT = 245; // degrees
  public static final double ARM_MINIMUM_ANGLE = -5; // degrees
  public static final double ARM_MAXIMUM_ANGLE = 265; // degrees
  public static final double ARM_RESET_PERCENT = -0.15; // percent to move arm back to hall effect
  public static final double ARM_TOLERANCE = 3; // +degrees
  public static final double ARM_P = 1.1;
  
  // intake
  public static final double INTAKE_PERCENT = 1.0; // 0.4
  public static final double INTAKE_PERCENT_BACK = -1.0;

  // camera
  public static final double CAMERA_TILT = 27; // degrees TODO Check camera tilt with Saxon
  public static final double ELEVATION = 20; // inches from floor to aperature of camera TODO Check camera elevation with Saxon

  // camera pid
  public static final double CAMERA_BIAS = -1; // degrees clockwise the turret should face from camera TODO check with saxon if 0 is better than -1
  public static final double APPROACH_SETPOINT = 120; // inches from target
  public static final double TOLERANCE_APPROACH = 2; // inches away from setpoint
  public static final double KP_APPROACH_LINEAR = 0.001; // TODO needs to be tuned
  public static final double KI_APPROACH_LINEAR = 0;
  public static final double KD_APPROACH_LINEAR = 0;
  public static final double KP_APPROACH_ANGULAR = 0.006;
  public static final double KI_APPROACH_ANGULAR = 0;
  public static final double KD_APPROACH_ANGULAR = 0;

  // pathing
  public static final double DRIVE_KS = 0.114; // Volts
  public static final double DRIVE_KV = 2.49; // Volts * s/m
  public static final double DRIVE_KA = 0.271; // Volts * s^2/m
  public static final double TRACK_WIDTH = 0.8127575527930411; // meters
  public static final double AUTO_PATH_KP = 0; // TODO needs to be tuned
  public static final DifferentialDriveKinematics KINEMATICS =
        new DifferentialDriveKinematics(TRACK_WIDTH);
  
  public static final double DRIVE_MAX_M_PER_S = 189.72441; // in/s
  public static final double DRIVE_MAX_M_PER_S_SQUARED = 1743; // in/s^2
  public static final double AUTO_MAX_M_PER_S = 118; // in/s
  public static final double AUTO_MAX_M_PER_S_SQUARED = 67; // in/s^2

  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  public static final double RAMSETE_B = 2;
  public static final double RAMSETE_ZETA = 0.7;

  // auto
  public static final double AUTO_LINEAR_TOLERANCE = 2; // inches
  public static final double AUTO_LINEAR_P = 0.03;
  public static final double AUTO_LINEAR_I = 0;
  public static final double AUTO_LINEAR_D = 0;
  public static final double AUTO_LINEAR_ANGLE_P = 0.02;

  public static final double AUTO_ANGULAR_TOLERANCE = 2; // degrees
  public static final double AUTO_ANGULAR_P = 0.01;
  public static final double AUTO_ANGULAR_I = 0;
  public static final double AUTO_ANGULAR_D = 0;

}
