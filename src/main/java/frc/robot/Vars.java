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
  public static final boolean SHOOTER_LEFT_REVERSED = false;
  public static final boolean SHOOTER_RIGHT_REVERSED = true;
  public static final boolean HOPPER_WALL_REVERSED = false;
  public static final boolean HOPPER_FLOOR_REVERSED = false;
  public static final boolean INTAKE_REVERSED = false;
  public static final boolean ARM_ROTATOR_REVERSED = true;
  public static final boolean ARM_INTAKE_REVERSED = false;
  public static final boolean CLIMB_CHAIN_INVERTED = false;
  public static final boolean CLIMB_ENCODER_INVERTED = false; 
  // TODO check to see if inverted is needed for intake, arm, and hopper
  
  // flipped encoders
  public static final boolean TURRET_ENCODER_REVERSED = false;
  public static final boolean SHOOTER_ENCODER_REVERSED = true;
  public static final boolean ARM_ENCODER_REVERSED = false;
  
  // climb 
  public static final double CLIMB_MAXIMUM_TARGET = 80; // inches 
  // 78.875 inches is the real maximum of the handle bar 
  public static final double CLIMB_UP_POSITION = 80;
  public static final double CLIMB_DOWN_POSITION = 80;
  public static final double CLIMB_MINIMUM_TARGET = 0; /** TODO Fill this in after we figure it out */ // inches
  // 50.25 inches is the real minimum of the handle bar
  public static final double CLIMB_P = 0.1;

  // turret
  public static final double TURRET_TOLERANCE = 10; // degrees
  
  // shooter
  public static final double SHOOTER_DEFAULT = 5000; // rpm
  public static final double SHOOTER_KP = 0.15; // TODO Further tune this value higher
  
  // intake
  public static final double INTAKE_LOW_FEED = 0.2;

  // hopper
  public static final double HOPPER_WALL_PERCENT = 0.2;
  public static final double HOPPER_FLOOR_PERCENT = 0.2;
  
  // arm
  public static final double ARM_P = 1.4; // TODO Maybe change these values? They're from last year
  public static final double ARM_MINIMUM_ANGLE = -3; // TODO Maybe change these values? They're from last year
  public static final double ARM_MAXIMUM_ANGLE = 157; // TODO Maybe change these values? They're from last year
  public static final double ARM_INTAKE = 0.2; // This one is new, just remembered it from yesterday

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
