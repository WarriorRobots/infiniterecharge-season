/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AimANDFire extends Command {
  // JOSE CODE JOSE CODE
  // shooter detects target 
  // WHERE IS TARGET OBJECT JOSHUA??
  private int far = 24 * 12;
  private int near = 9 * 12;

  public AimANDFire() {
    requires(Robot.camera);
    requires(Robot.drivetrain);
    requires(Robot.shooterpewpew);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }
  
  // JOSE CODE JOSE CODE
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    // if robot can see object
    if (Robot.camera.canSeeObject()) 
    {
      // rotate robot to object
      Robot.drivetrain.arcadeDriveRaw(0, Robot.camera.getObjectX());   
      // if robot is face the object then drive towards it 
      if(Math.abs(Robot.camera.getObjectX()) <= 0.1) 
      {
        Robot.drivetrain.arcadeDriveRaw(Robot.camera.getTargetDistance(), 0);
      }
      // if robot is next to object, FIRE AT WILL 
      if(Robot.camera.getTargetDistance() >= 108 && Robot.camera.getTargetDistance() <= 244)
      {
        Robot.shooterpewpew.setRPM(5000);
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
