/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.spinner;

import edu.wpi.first.wpilibj.command.Command;

public class SpinRotate extends Command {

  /**
   * This command spins the color wheel 4 rotations.
   */
  public SpinRotate() {}

  @Override
  protected void initialize() {}

  @Override
  protected void execute() {
    // Rotate the wheel three times
  }

  @Override
  protected boolean isFinished() {
    // returns whether the wheel has been spun 3 times
    return false;
  }

  @Override
  protected void end() {
    // stop the spinner
  }

  @Override
  protected void interrupted() {
    // stop the spinner
  }
}
