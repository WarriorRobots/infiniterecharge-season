/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;

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

  private List<RamseteCommand> ramsetes;

  private DrivetrainSubsystem m_drive;
  private List<TBase> m_bases;
  private List<Trajectory> m_trajectory;

  public RamseteContainer(DrivetrainSubsystem drive, TBase... trajectoryBases) {
    m_drive = drive;
    m_bases = List.of(trajectoryBases);
    m_trajectory = new ArrayList<Trajectory>();
    ramsetes = new ArrayList<RamseteCommand>();
    // m_trajectory = trajectoryBase.getTrajectory();
    // SmartDashboard.putNumber("Trajectory est.", m_trajectory.getTotalTimeSeconds()); // TODO remove debug

    for (TBase base : m_bases) {
      m_trajectory.add(base.getTrajectory());

      ramsetes.add(
        new RamseteCommand(
            base.getTrajectory(),
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
        )
      );
    }
  }

  /**
   * Gets the Ramsete Command
   * @return the command (and stops the drive after it finishes)
   */
  public CommandBase getCommand() {
    // // start with a blank instant command that does nothing and keep adding ramete commands on it
    // CommandBase allCommands = new InstantCommand();
    // for (RamseteCommand ramsete : ramsetes) {
    //   allCommands.andThen(ramsete);
    return new InstantCommand().andThen(ramsetes.toArray(new RamseteCommand[ramsetes.size()]));
  }
}
