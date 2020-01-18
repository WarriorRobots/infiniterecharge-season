/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TurretToPosition extends Command {

  private double target;
  private final double tolerance = 10; // TODO check what a good tolerance is

  /**
   * Rotates the target to the specified target rotation.
   * 
   * @param target The target the turret is to rotate to in degrees.
   */
  public TurretToPosition(double target) {
    requires(Robot.turret);
    this.target = target;
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.turret.rotateToPosition(target);
  }
  
  @Override
  protected boolean isFinished() {
    return Math.abs(target-Robot.turret.getRotationDegrees())<tolerance;
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
