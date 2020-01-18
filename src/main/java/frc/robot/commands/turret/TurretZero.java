/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.TurretSubsystem;

public class TurretZero extends Command {

  private double initialPosition;
  /** Amount to rotate off of initial to search for the 0 on any given loop */
  private double rotateByAmount;
  /** Amount extra to search each time the turret flips directions */
  private final double stepIncrement = 10;
  /** Direction the robot is currently turning, true is clockwise */
  private boolean direction;

  /**
   * Find the zero of the turret. Rotates back and forth (in incremental amounts) to try and find the 0 of the robot.
   */
  public TurretZero() {
   requires(Robot.turret);
  }
  
  @Override
  protected void initialize() {
    initialPosition = Robot.turret.getRotationDegrees();
    rotateByAmount = -initialPosition; // Start by heading back in the direction of the 0
    direction = rotateByAmount>=0;
  }
  
  @Override
  protected void execute() {
    // if turning clockwise and have passed the clockwise boundary
    if (direction && Robot.turret.getRotationDegrees()>initialPosition+rotateByAmount) {
      // go the other direction
      direction = !direction;
      // rotate amount is it's flip + the increment in the new direction
      rotateByAmount = -rotateByAmount + stepIncrement * (direction ? 1 : -1);
    }
    // if turning counterclockwise and have passed the counterclockwise boundary
    else if (!direction && Robot.turret.getRotationDegrees()<initialPosition+rotateByAmount) {
      // go the other direction
      direction = !direction;
      // rotate amount is it's flip + the increment in the new direction
      rotateByAmount = -rotateByAmount + stepIncrement * (direction ? 1 : -1);
    }
    Robot.turret.rotate(.1 * (direction ? 1 : -1));
  }
  
  @Override
  protected boolean isFinished() {
     // The robot should not have to rotate more than 1/2 the range to find it's home
    return Robot.turret.isCentered() || Math.abs(rotateByAmount)>TurretSubsystem.RANGE_ROTATION/2;
  }
  
  @Override
  protected void end() {
    Robot.turret.stop();
  }
  
  @Override
  protected void interrupted() {
    Robot.turret.stop();
  }
}
