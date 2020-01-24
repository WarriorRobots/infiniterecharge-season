/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.spinner;

import edu.wpi.first.wpilibj.command.Command;

public class SpinToColor extends Command {

  /** The color the field wants, set when the command is initialized. */
  String desiredColor;

  /**
   * A boolean to keep track whether the wheel is finding the color or is turning it so
   * the field sees the desired color.
   * True 
   */
  boolean foundColor;

  /**
   * Spins the color wheel to a specific color provided by the field.
   * The desired color will be set when the command is run since the field must provide the color.
   */
  public SpinToColor() {}

  @Override
  protected void initialize() {
    // get the color from the field.
    //desiredColor = <get from field>
  }

  @Override
  protected void execute() {
    // if the desired color is not found, rotate the wheel till it is found
    // if the desired color is found, rotate the wheel so the field sees the desired color

    // if the robot sees the desired color, the desired color is found
  }

  @Override
  protected boolean isFinished() {
    // True when the robot has made it so the field sensor can see the desired color
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
