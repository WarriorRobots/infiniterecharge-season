/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class AimANDFire extends CommandBase {
  ShooterSubsystem pewpew;

  public AimANDFire(ShooterSubsystem pewpew)
  {
    this.pewpew = pewpew;
    addRequirements(this.pewpew);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {}
  
  // JOSE CODE JOSE CODE
  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() 
  {
    // if robot can see object
    if (camera.canSeeObject()) 
    {
      // rotate robot to object
      Robot.drivetrain.arcadeDriveRaw(0, Robot.camera.getObjectX());   
      // if robot is face the object then drive towards it 
      if(Math.abs(Robot.camera.getObjectX()) <= 0.1) 
      {
        m_drivetrain.arcadeDriveRaw(Robot.camera.getTargetDistance(), 0);
      }
      // if robot is next to object, FIRE AT WILL 
      if(Math.abs(9 - Robot.camera.getTargetDistance()) <= 1)
      {
        Robot.shooterpewpew.setRPM(5000);
      }
    }
  } 


  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  
}
