/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.ClimbSubsystem;

class SubClimbToPosition extends CommandBase {

  ClimbSubsystem m_climb;

  double position;

  /**
   * A command to make the climb move to a set position. Ends after the climb reaches said position.
   */
  public SubClimbToPosition(ClimbSubsystem climb, double position) {
    m_climb = climb;
    addRequirements(m_climb);
    this.position = position;
  }

  @Override
  public void execute() {
    m_climb.setPosition(position);
  }

  @Override
  public void end(boolean interrupted) {
    // the climb needs to be stopped so that it does not run while the piston is out.
    m_climb.stopWinch();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_climb.getPosition() - position) < Vars.CLIMB_TOLERANCE;
  }
}
