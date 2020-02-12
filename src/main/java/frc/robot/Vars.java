package frc.robot;

/**
 * All variables tied to the physical behavior of the robot. Tuning, reverse booleans, etc.
 */
public class Vars {

	public static final double MAX_VELOCITY = 114; // inches/sec
	public static final double MAX_ACCELERATION = 220; // inches/sec^2

	public static final boolean LEFT_DRIVE_INVERTED = false;
	public static final boolean RIGHT_DRIVE_INVERTED = false;
	public static final boolean TURRET_REVERSED = false;
	public static final boolean TURRET_ENCODER_REVERSED = false;

  	// shooter
	public static final boolean SHOOTER_REVERSED = false;
	public static final boolean SHOOTER_ENCODER_REVERSED = true;
	public static final double SHOOTER_KP = 0.07;

	// camera
	public static final double CAMERA_TILT = 0;
	public static final double ELEVATION = 40; // in
	
	// camera pid
	public static final double APPROACH_SETPOINT = 10;
	public static final double KP_APPROACH_LINEAR = 0.030; // TODO tune for 2020 robot
	public static final double KI_APPROACH_LINEAR = 0;
	public static final double KD_APPROACH_LINEAR = 0;
	public static final double TOLERANCE_APPROACH = 2; // inches away from setpoint
	public static final double KP_APPROACH_ANGULAR = 0.055; // TODO tune for 2020 robot
	public static final double KI_APPROACH_ANGULAR = 0;
	public static final double KD_APPROACH_ANGULAR = 0;

}
