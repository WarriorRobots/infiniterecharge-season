/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterHopper extends CommandBase {

  ShooterSubsystem m_shooter;
  HopperSubsystem m_hopper;
  FeedSubsystem m_feed;

  boolean willPreShoot;
  boolean preShooting;
  // double time_requirement;

  /**
   * A command that runs the shooter and then when the shooter is up to speed, feeds the shooter.
   * @param willPreShoot If true, the robot will make sure there are no balls touching the shooter wheel. If no, it skips that step.
   */
  public ShooterHopper(ShooterSubsystem shooter, HopperSubsystem hopper, FeedSubsystem feed, boolean willPreShoot) {
    m_shooter = shooter;
    addRequirements(m_shooter);
    m_hopper = hopper;
    addRequirements(m_hopper);
    m_feed = feed;
    addRequirements(m_feed);

    this.willPreShoot = willPreShoot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    preShooting = willPreShoot;
    // time_requirement = Timer.getFPGATimestamp() + Vars.SHOOTER_PRETIME;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 2 stages of this command, a preshooting phase and a shooting phase
    if (preShooting) {
      // time to back the feed to not have balls touch the shooter

      // back balls up and don't run shooter
      m_shooter.stop();
      m_hopper.setWallPower(-0.2);
      m_hopper.setFloorPower(-0.2);
      m_feed.feedAtPercent(-0.2);

      // if the time requirement is met, it may start shooting
      // if (Timer.getFPGATimestamp()>time_requirement) {
      if (!m_feed.containsBall()) {
        preShooting = false;
        m_hopper.stop();
        m_feed.stop();
      }
    } else {
      // time for the shooter to rev and run

      if (Math.abs(m_shooter.getRPM()-m_shooter.getCommandedRPM()) < Vars.SHOOTER_TOLERANCE) {
        // if the shooter is fast enough, feed it
        m_hopper.setWallPower(Vars.HOPPER_WALL_PERCENT);
        m_hopper.setFloorPower(Vars.HOPPER_FLOOR_PERCENT);
        m_feed.feedAtPercent(Vars.FEED_PERCENT);
      } else {
        // if the shooter is not fast enough, do not feed it
        m_hopper.stop();
        m_feed.stop();
      }

      // run/rev the shooter
      m_shooter.setRPM(m_shooter.getCommandedRPM());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_hopper.stop();
    m_feed.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
