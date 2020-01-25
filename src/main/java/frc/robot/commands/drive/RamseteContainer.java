/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * A file to contain the Ramsete command to pass it into autonomous drive commands.
 */
public class RamseteContainer {

  // These four numbers come from a drive characterization
  // doing a drive characterization can be found here http://docs.wpilib.org/en/latest/docs/software/examples-tutorials/trajectory-tutorial/characterizing-drive.html
  public static final double ksVolts = 0.22;
  public static final double kvVoltSecondsPerMeter = 1.98;
  public static final double kaVoltSecondsSquaredPerMeter = 0.2;
  public static final double kPDriveVel = 8.5;

  // These Ramsete values are defaults and work for most robots
  private static final double kRamseteB = 2;
  private static final double kRamseteZeta = 0.7;

  private RamseteCommand ramsete;

  private DrivetrainSubsystem m_drive;
  private Trajectory m_trajectory;

  public RamseteContainer(DrivetrainSubsystem drive, Trajectory trajectory) {
    m_drive = drive;
    m_trajectory = trajectory;

    ramsete = new RamseteCommand(
        m_trajectory,
        m_drive::getPose,
        new RamseteController(kRamseteB, kRamseteZeta),
        new SimpleMotorFeedforward(ksVolts,
                                   kvVoltSecondsPerMeter,
                                   kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        m_drive::getWheelSpeeds,
        new PIDController(kPDriveVel, 0, 0),
        new PIDController(kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drive::tankdriveRaw,
        m_drive
    );
  }

  /**
   * Gets the Ramsete Command
   * @return the command (and stops the drive after it finishes)
   */
  public SequentialCommandGroup getCommand() {
    return ramsete.andThen(() -> m_drive.stop());
  }

}
