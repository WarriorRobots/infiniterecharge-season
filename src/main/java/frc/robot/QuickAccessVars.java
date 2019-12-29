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

	// pneumatic
	public static final double PNEUMATIC_LOOP_COUNT = 5; // # of loops pneumatics run

	// camera
	public static final double CAMERA_TILT = 0;
	public static final double ELEVATION = 40; // in
	public static final double CAMERA_DRIVE_THRESHOLD = 0.2; // decimal amount the driver has to push the joystick to activate arcade drive during Camera control

	public static final double MAX_VELOCITY = 114; // inches/sec XXX move to DrivetrainSubsystem

}
