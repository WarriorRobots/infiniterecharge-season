/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbBrakes extends CommandBase {

  ClimbSubsystem m_climb;
  DoubleSolenoid.Value value;
  int count;

  /**
   * A command to set the brakes.
   * @see ClimbSubsystem#setBrakes
   */
  public ClimbBrakes(ClimbSubsystem climb, DoubleSolenoid.Value value) {
    m_climb = climb;
    addRequirements(m_climb);
    this.value = value;
  }
  
  @Override
  public void initialize() {
    count = 0;
  }
  @Override
  public void execute() {
    m_climb.setBrakes(value);
    count++;
  }

  @Override
  public void end(boolean interrupted) {
    m_climb.stopBrakes();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count > Vars.PNEUMATIC_LOOP_COUNT;
  }
}
