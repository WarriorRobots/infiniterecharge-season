package frc.robot;

/**
 * All variables tied to the physical behavior of the robot. Tuning, reverse booleans, etc.
 */
public class Vars {
	
	public static final double MAX_VELOCITY = 114; // inches/sec
	public static final double MAX_ACCELERATION = 220; // inches/sec^2

  	public static final boolean LEFT_DRIVE_INVERTED = false;
  	public static final boolean RIGHT_DRIVE_INVERTED = false;

  	// shooter
	public static final boolean SHOOTER_REVERSED = false;
	public static final boolean SHOOTER_ENCODER_REVERSED = true;
	public static final double SHOOTER_KP = 0.07;
	

	public static final double INTAKE_LOW_FEED = 0.2;

	// arm
	public static final boolean ARM_ROTATOR_INVERTED = true;
	public static final boolean ARM_ENCODER_INVERTED = false;
	public static final double ARM_P = 1.4; // TODO Maybe change these values? They're from last year
	public static final double ARM_MINIMUM_ANGLE = -3; // TODO Maybe change these values? They're from last year
	public static final double ARM_MAXIMUM_ANGLE = 157; // TODO Maybe change these values? They're from last year
	public static final double ARM_INTAKE = 0.2; // This one is new, just remembered it from yesterday




}
