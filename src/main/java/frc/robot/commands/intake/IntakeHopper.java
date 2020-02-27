/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeHopper extends CommandBase {

  IntakeSubsystem m_intake;
  HopperSubsystem m_hopper;
  FeedSubsystem m_feed;
  /**
   * Command to run intake continously and hopper with intake until the intake has a ball.
   */
  public IntakeHopper(IntakeSubsystem intake, HopperSubsystem hopper, FeedSubsystem feed) {
    m_intake = intake;
    addRequirements(m_intake);
    m_hopper = hopper;
    addRequirements(m_hopper);
    m_feed = feed;
    addRequirements(m_feed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.intakeAtPercent(Vars.INTAKE_PERCENT);
    if (!m_feed.containsBall()) {
      // only run the hopper and feed while there is not a ball present in the feed
      m_hopper.setFloorPower(Vars.HOPPER_FLOOR_PERCENT);
      m_hopper.setWallPower(Vars.HOPPER_WALL_PERCENT);
      m_feed.feedAtPercent(Vars.FEED_PERCENT);
    } else {
      // if a ball is in the feed, there is no need to run it any more
      m_hopper.stop();
      m_feed.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_hopper.stop();
    m_feed.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
