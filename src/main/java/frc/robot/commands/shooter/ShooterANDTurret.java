/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;;

/**
 * start heading is zero
 * as robot rotates it increases (left) decreases (right)
 * making the turret rotate, not the robot
 *  frame doesn't move
 * if frame is off, then rotate shooter
 */

public class ShooterANDTurret extends Command {
  public ShooterANDTurret() {
    requires(Robot.camera);
    requires(Robot.shooterpewpew);
    requires(Robot.turret);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.camera.canSeeObject()) 
    {
      // rotate TURRET to object
      Robot.turret.rotateToPosition(Robot.camera.getObjectX());;   
      // if TURRET is facing the object AND is within range then FIRE AT WILL 
      if((Math.abs(Robot.camera.getObjectX()) <= 0.1) && Math.abs(9 - Robot.camera.getTargetDistance()) <= 1)
      {
        
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
