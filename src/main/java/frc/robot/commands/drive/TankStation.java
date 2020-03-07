/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TankStation extends CommandBase {

  DrivetrainSubsystem m_drive;
  TurretSubsystem m_turret;
  CameraSubsystem m_camera;

  private PIDController PIDcenter;

  DoubleSupplier m_left, m_right;

  public TankStation(DrivetrainSubsystem drive, TurretSubsystem turret, CameraSubsystem camera, DoubleSupplier left, DoubleSupplier right) {
    m_drive = drive;
    addRequirements(m_drive);
    m_turret = turret;
    addRequirements(m_turret);
    m_camera = camera;
    addRequirements(m_camera);
    m_left = left;
    m_right = right;

    PIDcenter = new PIDController(Vars.AUTO_ANGULAR_P, 0, 0);
  }

  @Override
  public void initialize() {
    m_camera.setPipeline(CameraSubsystem.PIPELINE_HEX);

    PIDcenter.setSetpoint(0); // keep the target in the center of the screen
  }

  @Override
  public void execute() {
    m_turret.rotateToPosition(0); // face forwards

    double valueCenter;
    valueCenter = PIDcenter.calculate(m_camera.getObjectX()+Vars.CAMERA_BIAS_STATION);
    if (!m_camera.canSeeObject()) {
      valueCenter = 0;
    }

    //valueApproach = Robot.input.getRightY();
    double valueApproach = ( m_left.getAsDouble() + m_right.getAsDouble() ) / 2;

    // the valueCenter needs to be negative because the camera object x and bias are postive and to minimize this it has to go the opposite way
    m_drive.arcadedriveRaw(valueApproach, -valueCenter);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean inturrupted) {
    m_camera.setPipeline(CameraSubsystem.PIPELINE_DRIVER);
		PIDcenter.reset();
		m_drive.stop();
  }
}