/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class FeedBall extends CommandBase {
  /**
   * Creates a new feedBall.
   */
  IntakeSubsystem m_hungryhippo;
  DoubleSupplier m_feed;
  /**
   * Creates a new setHopperPower.
   */
  public FeedBall(IntakeSubsystem hungryhippo, DoubleSupplier feed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_hungryhippo = hungryhippo;
    addRequirements(this.m_hungryhippo);
    m_feed = feed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hungryhippo.feedAtPercent(m_feed.getAsDouble());
    m_hungryhippo.lowFeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
