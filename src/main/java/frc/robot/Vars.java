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

	// camera
	public static final double CAMERA_TILT = 0;
	public static final double ELEVATION = 40; // in
	public static final double CAMERA_DRIVE_THRESHOLD = 0.2; // decimal amount the driver has to push the joystick to activate arcade drive during Camera control
	
	// camera pid
	public static final double KP_APPROACH = 0.030; // TODO tune for 2020 robot
	public static final double KI_APPROACH = 0;
	public static final double KD_APPROACH = 0.08; // TODO tune for 2020 robot
	public static final double SETPOINT_APPROACH = 15; // distance in inches the robot will attempt to stop from the target TODO tune for 2020 season
	public static final double TOLERANCE_APPROACH = 2; // tolerance in inches for the leds to show confirmation the robot is at the setpoint
	public static final double KP_CENTER = 0.055; // TODO tune for 2020 robot
	public static final double KI_CENTER = 0;
	public static final double KD_CENTER = 0;
	public static final double CAMERA_BIAS = 0.0; // amount of degrees added to the center the target when driving in

}
