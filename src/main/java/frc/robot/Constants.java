package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * All <i>actual</i> constants are here, field descriptions or things about the code that don't change.
 */
public final class Constants {
    
    public static final int PRIMARY_PID = 0; // primary pid ids
    public static final int AUXILARY_PID = 1; // auxilary pid ids
    public static final int MS_TIMEOUT = 10; // 10 ms before a talon config fails

    public static final double kTrackwidthMeters = Units.inchesToMeters(26);
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    

}
