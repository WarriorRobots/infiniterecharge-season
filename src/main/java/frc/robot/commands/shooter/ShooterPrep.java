/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPrep extends CommandBase {

  ShooterSubsystem m_shooter;
  HopperSubsystem m_hopper;
  FeedSubsystem m_feed;

  /**
   * A command to clear the shooter of any balls (usually before shooting is ran.)
   */
  public ShooterPrep(ShooterSubsystem shooter, HopperSubsystem hopper, FeedSubsystem feed) {
    m_shooter = shooter;
    addRequirements(m_shooter);
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
    m_hopper.setWallPower(Vars.SHOOTER_PRE);
    m_hopper.setFloorPower(Vars.SHOOTER_PRE);
    m_feed.feedAtPercent(Vars.SHOOTER_PRE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.stop();
    m_feed.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // make sure a ball is not in the shooter
    return !m_feed.containsBall();
  }
}
