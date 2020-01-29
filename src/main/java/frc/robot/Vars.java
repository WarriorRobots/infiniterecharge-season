package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * All variables tied to the physical behavior of the robot. Tuning, reverse booleans, etc.
 */
public class Vars {

  // These four numbers come from a drive characterization
  // doing a drive characterization can be found here http://docs.wpilib.org/en/latest/docs/software/examples-tutorials/trajectory-tutorial/characterizing-drive.html
  public static final double ksVolts = 0.583;
  public static final double kvVoltSecondsPerMeter = 6.97;
  public static final double kaVoltSecondsSquaredPerMeter = 0.99;
  // r-squared = 0.999
  public static final double kPDriveVel = 51.8;

  // The Ramsete values of 2 and 0.7 are defaults and work for most robots
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;


	public static final double MAX_VELOCITY = 114; // inches/sec
	public static final double MAX_ACCELERATION = 220; // inches/sec^2

  public static final boolean LEFT_DRIVE_INVERTED = false;
  public static final boolean RIGHT_DRIVE_INVERTED = false;
  

  public static final double kTrackwidthMeters = Units.inchesToMeters(26);
  public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(kTrackwidthMeters);

}
