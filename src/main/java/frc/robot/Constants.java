package frc.robot;

/**
 * All <i>actual</i> constants are here, field descriptions or things about the code that don't change.
 */
public final class Constants {
	
    public static final int PRIMARY_PID = 0; // primary pid ids
    public static final int AUXILARY_PID = 1; // auxilary pid ids
    public static final int MS_TIMEOUT = 10; // 10 ms before a talon config fails

	// camera (Limelight 2)
	public static final double PIXELS_H = 320;
	public static final double PIXELS_V = 240;
	public static final double RAD_H = 1.04;
	public static final double RAD_V = 0.867;
	public static final double PPR_H = PIXELS_H / RAD_H; // Pixels per Radian
	public static final double PPR_V = PIXELS_V / RAD_V; // Pixels per Radian


	// field elements
	public static final double TARGET_DEPTH = 29.25; // in
	// how high the circle is from the ground
	public static final double TARGET_CIRCLE_HEIGHT = 98.25; // in
	public static final double TARGET_ELEVATION = 81.25; // in
	public static final double TARGET_WIDTH = 39.25; // in
    public static final double TARGET_HEIGHT = 17; // in
	/** Ratio of width/height that determines whether the camera is cutting the bottom of the target off; {@value}. */
	public static final double CUTTING_RATIO = TARGET_WIDTH/TARGET_HEIGHT;

	public static final double PORT_HEIGHT = 17; // inches
	/** Height of the energy port */
	public static final double PORT_WIDTH = 39.25; // inches
	/** Width of the energy port */
	public static final int PICKUP_HEIGHT = 11; // inches
	/** Height of the ball pickup port */
	public static final int PICKUP_WIDTH = 7; // inches
	/** Width of the ball pickup port */
	public static final double PORT_ASPECT_RATIO = PORT_WIDTH / PORT_HEIGHT;
	/** Aspect ratio (width/height) of the port */
	public static final double PICKUP_ASPECT_RATIO = PICKUP_WIDTH / PICKUP_HEIGHT;
	/** The distance from the target to the camera in terms of pixels (for the limelight) */
	private static int FoVWidth = 320;
	private static double FoVAngel = 44.5;
	public static final double PIXEL_DISTANCE = (FoVWidth/2)/Math.tan(FoVAngel/2);

	/**
	 * ID number for the pnuematic
	 */
	public static final int PCM_1 = 1; // TODO change this number later Joshua and put in RobotMap.java
}
