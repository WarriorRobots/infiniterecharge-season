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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Vars;
import frc.robot.commands.drive.trajectories.TBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * A file to contain the Ramsete command to pass it into autonomous drive commands.
 */
public class RamseteContainer {

  private RamseteCommand ramsete;

  private DrivetrainSubsystem m_drive;
  private TBase m_base;
  private Trajectory m_trajectory;

  public RamseteContainer(DrivetrainSubsystem drive, TBase trajectoryBase) {
    m_drive = drive;
    m_base = trajectoryBase;
    m_trajectory = trajectoryBase.getTrajectory();
    SmartDashboard.putNumber("Trajectory est.", m_trajectory.getTotalTimeSeconds()); // TODO remove debug

    ramsete = new RamseteCommand(
        m_trajectory,
        m_drive::getPose,
        new RamseteController(Vars.kRamseteB, Vars.kRamseteZeta),
        new SimpleMotorFeedforward(Vars.ksVolts,
                                   Vars.kvVoltSecondsPerMeter,
                                   Vars.kaVoltSecondsSquaredPerMeter),
        Vars.kDriveKinematics,
        m_drive::getWheelSpeeds,
        new PIDController(Vars.kPDriveVel, 0, 0),
        new PIDController(Vars.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drive::tankdriveVoltage,
        m_drive
    );
  }

  /**
   * Gets the Ramsete Command
   * @return the command (and stops the drive after it finishes)
   */
  public CommandBase getCommand() {
    if (m_base.endSpeed() == 0) return ramsete.andThen(new InstantCommand(m_drive::stop, m_drive));
    else return ramsete;
  }
}
