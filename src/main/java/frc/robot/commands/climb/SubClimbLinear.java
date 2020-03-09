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

class SubClimbLinear extends CommandBase {

  ClimbSubsystem m_climb;
  DoubleSupplier input;

  /**
   * A command to tell the climb to move linearly.
   */
  public SubClimbLinear(ClimbSubsystem climb, DoubleSupplier input) {
    m_climb = climb;
    addRequirements(m_climb);
    this.input = input;
  }

  @Override
  public void execute() {
    m_climb.setPercent(input.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_climb.stopWinch();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
