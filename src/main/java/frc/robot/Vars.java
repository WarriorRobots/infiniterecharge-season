package frc.robot;

/**
 * All variables tied to the physical behavior of the robot. Tuning, reverse booleans, etc.
 */
public class Vars {
  
  // robot characteristics
  public static final double WHEEL_DIAMETER = 6; // inches
  public static final double MAX_VELOCITY = 114; // inches/sec
  public static final double MAX_ACCELERATION = 220; // inches/sec^2
  
  // flipped motors
  public static final boolean LEFT_DRIVE_INVERTED = false;
  public static final boolean RIGHT_DRIVE_INVERTED = false;
  public static final boolean TURRET_REVERSED = false;
  public static final boolean SHOOTER_LEFT_REVERSED = true;
  public static final boolean SHOOTER_RIGHT_REVERSED = false;
  public static final boolean HOPPER_WALL_REVERSED = true;
  public static final boolean HOPPER_FLOOR_REVERSED = false;
  public static final boolean FEED_REVERSED = true;
  public static final boolean ARM_REVERSED = true;
  public static final boolean INTAKE_REVERSED = false;
  
  // flipped encoders
  public static final boolean TURRET_ENCODER_REVERSED = false;
  public static final boolean SHOOTER_ENCODER_REVERSED = true;
  public static final boolean ARM_ENCODER_REVERSED = false;
  
  // turret
  public static final double TURRET_TOLERANCE = 10; // degrees
  
  // shooter
  public static final double SHOOTER_DEFAULT = 5000; // rpm
  public static final double SHOOTER_KP = 0.15; // TODO Further tune this value higher
  
  // intake
  public static final double FEED_PERCENT = 1.0;

  // hopper
  public static final double HOPPER_WALL_PERCENT = 0.5;
  public static final double HOPPER_FLOOR_PERCENT = 1.0;
  
  // arm
  public static final double ARM_P = 1.4; // TODO Maybe change these values? They're from last year
  public static final double ARM_MINIMUM_ANGLE = -3; // TODO Maybe change these values? They're from last year
  public static final double ARM_MAXIMUM_ANGLE = 157; // TODO Maybe change these values? They're from last year
  
  // intake
  public static final double INTAKE_PERCENT = 0.4;

  // camera
  public static final double CAMERA_TILT = 27; // degrees TODO Check camera tilt with Saxon
  public static final double ELEVATION = 20; // inches from floor to aperature of camera TODO Check camera elevation with Saxon

  // camera pid
  public static final double CAMERA_BIAS = -1; // degrees clockwise
  public static final double APPROACH_SETPOINT = 120; // inches from target
  public static final double TOLERANCE_APPROACH = 2; // inches away from setpoint
  public static final double KP_APPROACH_LINEAR = 0.001; // TODO needs to be tuned
  public static final double KI_APPROACH_LINEAR = 0;
  public static final double KD_APPROACH_LINEAR = 0;
  public static final double KP_APPROACH_ANGULAR = 0.006;
  public static final double KI_APPROACH_ANGULAR = 0;
  public static final double KD_APPROACH_ANGULAR = 0;

}
