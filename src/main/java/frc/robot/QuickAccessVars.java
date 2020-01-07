package frc.robot;

/** Anything you need to access and edit quickly */ // @formatter:off
public final class QuickAccessVars {
	
	private QuickAccessVars() {}

	// input
	public static final double XBOX_JOYSTICK_THRESHOLD = 0.7; // how far the xbox joystick has to be pushed to trigger the digital inputs thresholding
	public static final double TURNLOCK_THRESHOLD = 0.2; // decimal difference allowed that joysticks considered are the "same" in turnlock mode

	// drivetrain
	public static final double DRIVETRAIN_RAMPRATE = 0.25; // time to go from 0 to full on a motor
	public static final boolean LEFT_SIDE_REVERSED = false;
	public static final boolean RIGHT_SIDE_REVERSED = false;
	public static final boolean LEFT_SIDE_ENCODER_REVERSED = false;
	public static final boolean RIGHT_SIDE_ENCODER_REVERSED = true;
	public static final double ARCADE_FORWARD_MODIFIER = 1; // percent as decimal modifier on the forwards motion input during arcade mode
	public static final double ARCADE_TURN_MODIFIER = 1;// percent as decimal modifier on the rotational motion input during arcade mode

	// pneumatic
	public static final int PNEUMATIC_LOOP_COUNT = 5; // # of loops pneumatics run

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
