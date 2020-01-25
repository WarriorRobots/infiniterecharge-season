/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.spinner;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ColorSpinnerSubsystem;

public class SpinRotate extends Command {

  /**
   * This command spins the color wheel 4 rotations.
   * (This means it will pass by the same start color 8 times.)
   */
  int count;
  String startingColor;
  boolean spunAlready;
  public SpinRotate() {
    requires(Robot.whee);
  
  }

  @Override
  protected void initialize() {
    startingColor = Robot.whee.getColor();
    count = 0;
  }

  @Override
  protected void execute() {
    // Rotate the wheel four times
    if (count != 8)
    {
      Robot.whee.spinWheel(0.25);
      if (Robot.whee.getColor() == startingColor && !spunAlready) // spunAlready != true is the same thing 
      {
        spunAlready = true;
        count++;
      }
      if (Robot.whee.getColor() != startingColor)
      {
        spunAlready = false;
      }
    }
  }

  @Override
  protected boolean isFinished() {
    // returns whether the wheel has been spun 3 times
      return count >= 8;
  }
  
  // JOSE CODE JOSE CODE

  @Override
  protected void end() {
    // stop the spinner
  }

  @Override
  protected void interrupted() {
    // stop the spinner
  }
}
