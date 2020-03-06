/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class LinearClimb extends CommandBase {
  /**
   * Creates a new LinearClimb.
   * I believe this is what allows us to manually comtrol the climb
   */
  ClimbSubsystem m_climb;
  DoubleSupplier m_percent;
  public LinearClimb(ClimbSubsystem climb, DoubleSupplier percent) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climb = climb;
    addRequirements(this.m_climb);
    m_percent = percent;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climb.climbAtPercent(m_percent.getAsDouble());

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