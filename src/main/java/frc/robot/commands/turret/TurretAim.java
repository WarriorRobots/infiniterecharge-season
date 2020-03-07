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
  CameraSubsystem m_camera;
  TurretSubsystem m_turret;

  /**
   * A command to aim the target at the hexagonal target.
   * (Also turns on the limelight LEDs and then turns them off after it finishes)
   */
  public TurretAim(CameraSubsystem camera, TurretSubsystem turret) {
    m_camera = camera;
    m_turret = turret;
    addRequirements(this.m_turret);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_camera.setPipeline(CameraSubsystem.PIPELINE_HEX);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (!m_camera.canSeeObject()) return;

    m_turret.rotateBounded(m_turret.getRotationDegrees() + m_camera.getObjectX() + Vars.CAMERA_BIAS_SHOOTER);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return m_camera.canSeeObject() && Math.abs(m_camera.getObjectX() + Vars.CAMERA_BIAS_SHOOTER) < Vars.TURRET_TOLERANCE;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_camera.setPipeline(CameraSubsystem.PIPELINE_DRIVER);
  }
}
