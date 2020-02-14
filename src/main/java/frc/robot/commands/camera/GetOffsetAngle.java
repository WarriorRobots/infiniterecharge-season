/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.camera;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants;

@Deprecated
public class GetOffsetAngle extends CommandBase {
 
/**
 * start heading is zero
 * as robot rotates it increases (left) decreases (right)
 * making the turret rotate, not the robot
 *  frame doesn't move
 * if frame is off, then rotate shooter
 */

  CameraSubsystem m_snapsnap;
  ShooterSubsystem m_pewpew;
  TurretSubsystem m_clank;

  public GetOffsetAngle(CameraSubsystem snapsnap, ShooterSubsystem pewpew, TurretSubsystem clank) {
    m_snapsnap = snapsnap;
    addRequirements(this.m_snapsnap);
    m_pewpew = pewpew;
    addRequirements(this.m_pewpew);
    m_clank = clank;
    addRequirements(this.m_clank);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }
  
  

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // variable set up
    double a = m_snapsnap.getTargetDistance(); 
    // target distance used in the law of cosines to find the inner target distanc[e]
    double b = Constants.TARGET_DEPTH; 
    // depth of the hole used in both the law of cosines to find the inner target distance and the offset angle
    double C = Math.PI + Math.asin(Constants.TARGET_WIDTH / m_snapsnap.getObjectWidth()); 
    // this is the angle the robot is to the target and is used in the law of sines to find the offset angle
    double c = Math.sqrt(Math.pow(a,2) + Math.pow(b,2) - 2 * a * b * Math.cos(C)); 
    // this is the distance to the inner target and is used in the law of sines to find the offset angle
    double B = Math.asin(Math.sin(C) * c / b);
    // offset angle of the actual target, what we need to rotate by
    double offsetAngle = B;
    // variables end

    m_snapsnap.getTargetDistance();
    if (m_snapsnap.canSeeObject())
    {
      if(m_snapsnap.getTargetDistance() >= 108 && m_snapsnap.getTargetDistance() <= 244)

      /** TODO multiply getObjectX() by 1/2 of the field of vision */
      m_clank.rotateToPosition(m_clank.getRotationDegrees() + m_snapsnap.getObjectX());
      // slightly turn the turret by the offsetAngle
      // figure out if it goes left or right(?)
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

}
