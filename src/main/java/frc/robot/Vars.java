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

}
