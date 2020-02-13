/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.KitDriveSubsystem;

public class TankDrive extends CommandBase {
  KitDriveSubsystem m_drive;
  DoubleSupplier m_left, m_right;

  /**
   * Creates a new TankDrive.
   * @param drive drivetrain subsystem
   * @param left Left side value supplier
   * @param right right side value supplier
   */
  public TankDrive(KitDriveSubsystem drive, DoubleSupplier left, DoubleSupplier right) {
    m_drive = drive;
    m_left = left;
    m_right = right;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    m_drive.tankDriveRaw(m_left.getAsDouble(), m_right.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
