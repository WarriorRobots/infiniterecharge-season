/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimANDFire extends CommandBase {
  ShooterSubsystem m_pewpew;
  DrivetrainSubsystem m_drive;
  CameraSubsystem m_snapsnap;

  public AimANDFire(ShooterSubsystem pewpew, DrivetrainSubsystem drive, CameraSubsystem snapsnap )
  {
    m_pewpew = pewpew;
    addRequirements(this.m_pewpew);
    m_drive = drive;
    addRequirements(this.m_drive);
    m_snapsnap = snapsnap;
    addRequirements(this.m_snapsnap);
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
    if (m_snapsnap.canSeeObject()) 
    {
      // rotate robot to object
      /** TODO FIX THIS JOSHUA */
      // m_drive.arcadeDriveRaw(0, m_snapsnap.getObjectX());   
      // if robot is face the object then drive towards it 
      if(Math.abs(m_snapsnap.getObjectX()) <= 0.1) 
      {
        /** TODO FIX THIS AS WELL JOSHUA */
        // m_drive.arcadeDriveRaw(m_snapsnap.getTargetDistance(), 0);
      }
      // if robot is next to object, FIRE AT WILL 
      if(Math.abs(9 - m_snapsnap.getTargetDistance()) <= 1)
      {
        m_pewpew.setRPM(5000);
      }
    }
  } 


  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  
}
