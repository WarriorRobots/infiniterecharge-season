/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.TurretSubsystem;


/**
 * start heading is zero
 * as robot rotates it increases (left) decreases (right)
 * making the turret rotate, not the robot
 *  frame doesn't move
 * if frame is off, then rotate shooter
 */

public class TurretAim extends CommandBase {
  CameraSubsystem m_snapsnap;
  TurretSubsystem m_clank;
  public TurretAim(CameraSubsystem snapsnap, TurretSubsystem clank) {
    m_snapsnap = snapsnap;
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
    if (!m_snapsnap.canSeeObject()) return;

    m_clank.rotateBounded(m_clank.getRotationDegrees() + m_snapsnap.getObjectX() + Vars.CAMERA_BIAS);
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
