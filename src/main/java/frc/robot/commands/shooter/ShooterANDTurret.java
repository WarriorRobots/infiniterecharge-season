/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Constants;


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
    // variable set up
    double a = Robot.camera.TargetDistance("port"); 
    // target distance used in the law of cosines to find the inner target distanc[e]
    double b = Constants.TARGET_DEPTH; 
    // depth of the hole used in both the law of cosines to find the inner target distance and the offset angle
    double C = Math.PI + Math.asin(Constants.TARGET_WIDTH / Robot.camera.getObjectWidth()); 
    // this is the angle the robot is to the target and is used in the law of sines to find the offset angle
    double c = Math.sqrt(Math.pow(a,2) + Math.pow(b,2) - 2 * a * b * Math.cos(C)); 
    // this is the distance to the inner target and is used in the law of sines to find the offset angle
    double B = Math.asin(Math.sin(C) * c / b);
    // offset angle of the actual target, what we need to rotate by
    double offsetAngle = B;
    // variables end

    Robot.camera.TargetDistance("port");
    if (Robot.camera.canSeeObject())
    {
      if(Robot.camera.TargetDistance("port") >= 108 && Robot.camera.TargetDistance("port") <= 244)

      /** TODO multiply getObjectX() by 1/2 of the field of vision */
      Robot.turret.rotateToPosition(Robot.camera.getObjectX() + offsetAngle);
      // slightly turn the turret by the offsetAngle
      // figure out if it goes left or right(?)
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
