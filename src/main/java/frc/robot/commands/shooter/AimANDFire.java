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
import frc.robot.subsystems.TurretSubsystem;

public class AimANDFire extends CommandBase {
  ShooterSubsystem m_shooter;
  TurretSubsystem m_turret;
  CameraSubsystem m_camera;

  public AimANDFire(ShooterSubsystem shooter, TurretSubsystem turret, CameraSubsystem camera)
  {
    m_shooter = shooter;
    addRequirements(this.m_shooter);
    m_turret = turret;
    addRequirements(this.m_turret);
    m_camera = camera;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {}
  
  // JOSE CODE JOSE CODE
  // thanks - Josh
  // Called repeatedly when this Command is scheduled to run
  
  @Override
  public void execute() 
  {
    // if robot can see object
    if (m_camera.canSeeObject()) 
    {
      // rotate turret to object
      m_turret.rotateBounded(m_turret.getRotationDegrees() + m_camera.getObjectX());

      if(Math.abs(m_camera.getObjectX()) <= 5) // if within 5 degrees of facing the target
      {
        m_shooter.setRPM(m_shooter.getCommandedRPM());
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
