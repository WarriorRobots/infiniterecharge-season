/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretZero extends CommandBase {
  TurretSubsystem m_clank;
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
  public TurretZero(TurretSubsystem clank) {
    m_clank = clank;
    addRequirements(m_clank);
  }
  
  @Override
  public void initialize() {
    initialPosition = m_clank.getRotationDegrees();
    rotateByAmount = -initialPosition; // Start by heading back in the direction of the 0
    direction = rotateByAmount>=0;
  }
  
  @Override
  public void execute() {
    // if turning clockwise and have passed the clockwise boundary
    if (direction && m_clank.getRotationDegrees()>initialPosition+rotateByAmount) {
      // go the other direction
      direction = !direction;
      // rotate amount is it's flip + the increment in the new direction
      rotateByAmount = -rotateByAmount + stepIncrement * (direction ? 1 : -1);
    }
    // if turning counterclockwise and have passed the counterclockwise boundary
    else if (!direction && m_clank.getRotationDegrees()<initialPosition+rotateByAmount) {
      // go the other direction
      direction = !direction;
      // rotate amount is it's flip + the increment in the new direction
      rotateByAmount = -rotateByAmount + stepIncrement * (direction ? 1 : -1);
    }
    m_clank.rotate(.1 * (direction ? 1 : -1));
  }
  
  @Override
  public boolean isFinished() {
     // The robot should not have to rotate more than 1/2 the range to find it's home
    return m_clank.isCentered() || Math.abs(rotateByAmount)>TurretSubsystem.RANGE_ROTATION/2;
  }
  
  @Override
  public void end(boolean interrupted) {
    m_clank.stop();
  }
}
