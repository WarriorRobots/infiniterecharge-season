package frc.robot;

/**
 * Contains all mathematical constants and calculations.
 */
public final class Constants {
	
	private Constants() {}
	
	/**
	 * ID number ({@value}) of the main PCM, which has the compressor and pressure
	 * switch plugged in.
	 */
	public static final int PCM_1 = 1;
	// TODO add extra solenoids as needed

	/**
	 * Profile number for any PID loop running on a Talon SRX.
	 * <p>
	 * Use in methods such as
	 * <code>configSelectedFeedbackSensor(FeedbackSensor.QuadEncoder, Constants.PID_ID, Constants.TIMEOUT_MS)</code>
	 */
	public static final int PID_ID = 0;

	/**
	 * How long a Talon PID loop waits for signals before terminating for safety.
	 * <p>
	 * Use in methods such as
	 * <code>configSelectedFeedbackSensor(FeedbackSensor.QuadEncoder, Constants.PID_ID, Constants.TIMEOUT_MS)</code>
	 */
	public static final int TIMEOUT_MS = 10;

	// camera (Limelight 2)
	public static final double PIXELS_H = 320;
	public static final double PIXELS_V = 240;
	public static final double RAD_H = 1.04;
	public static final double RAD_V = 0.867;
	public static final double PPR_H = PIXELS_H / RAD_H; // Pixels per Radian
	public static final double PPR_V = PIXELS_V / RAD_V; // Pixels per Radian

	// field elements TODO update all field elements for the 2020 game

	// JOSE CODE JOSE CODE
	// how far back the circle is
	public static final double TARGET_DEPTH = 29.25; // in
	// how high the circle is from the ground
	public static final double TARGET_CIRCLE_HEIGHT = 98.25; // in
	public static final double TARGET_ELEVATION = 81.25; // in
	public static final double TARGET_WIDTH = 39.25; // in
	public static final double TARGET_HEIGHT = 17; // in
	/** Ratio of width/height that determines whether the camera is cutting the bottom of the target off; {@value}. */
	public static final double CUTTING_RATIO = TARGET_WIDTH/TARGET_HEIGHT;

}
