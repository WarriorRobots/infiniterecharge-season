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
	public static final double TARGET_CIRCLE_HEIGHT = 98.25; // in
	public static final double TARGET_ELEVATION = 81.25; // in
	public static final double TARGET_WIDTH = 39.25; // in
    public static final double TARGET_HEIGHT = 17; // in
    
	/** Ratio of width/height that determines whether the camera is cutting the bottom of the target off; {@value}. */
	public static final double CUTTING_RATIO = TARGET_WIDTH/TARGET_HEIGHT;

}
