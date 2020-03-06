/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class DriveToDistance extends CommandBase {

  DrivetrainSubsystem m_drive;
  TurretSubsystem m_turret;
  CameraSubsystem m_camera;

  PIDController m_pid_linear;
  PIDController m_pid_angular;

  double target_distance;

  /**
   * Command that drives to a specific distance from the target.
   * (Also snaps the the turret to the target.)
   * @param distance distance in inches.
   */
  public DriveToDistance(DrivetrainSubsystem drive, TurretSubsystem turret, CameraSubsystem camera, double distance) {
    m_drive = drive;
    m_turret = turret;
    m_camera = camera;
    target_distance = distance;

    m_pid_linear = new PIDController(
      Vars.KP_APPROACH_LINEAR,
      Vars.KI_APPROACH_LINEAR,
      Vars.KD_APPROACH_LINEAR
    );
    m_pid_linear.setSetpoint(target_distance);
    
    m_pid_angular = new PIDController(
      Vars.KP_APPROACH_ANGULAR,
      Vars.KI_APPROACH_ANGULAR,
      Vars.KD_APPROACH_ANGULAR
    );
    m_pid_linear.setSetpoint(0);

    addRequirements(m_drive);
    addRequirements(m_turret);
  }

  @Override
  public void execute() {
    if (m_camera.canSeeObject())
    {
      // force the turret to face the target
      m_turret.rotateBounded(m_turret.getRotationDegrees() + m_camera.getObjectX());

      m_drive.arcadedriveRaw(
        -m_pid_linear.calculate(m_camera.getTargetDistance()),
        m_pid_angular.calculate(m_camera.getObjectX() - m_turret.getRelativeDegrees())
      );
      SmartDashboard.putNumber("X command", -m_pid_linear.calculate(m_camera.getTargetDistance())); // TODO remove debug
      SmartDashboard.putNumber("Z command", m_pid_angular.calculate(m_camera.getObjectX() - m_turret.getRelativeDegrees())); // TODO remove debug
      SmartDashboard.putNumber("Target relative to robot", m_camera.getObjectX() - m_turret.getRelativeDegrees()); // TODO remove debug
    } else {
      m_drive.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
    m_drive.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(target_distance-m_camera.getTargetDistance())<Vars.TOLERANCE_APPROACH;
  }
}
