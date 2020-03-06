/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TankStraight extends CommandBase {
  DrivetrainSubsystem m_drive;
  DoubleSupplier m_left, m_right;

  /**
   * Creates a new TankStraight.
   * This makes the robot automatically adjust to be straight if the driver commands the motors similarly.
   * @param drive drivetrain subsystem
   * @param left Left side value supplier
   * @param right right side value supplier
   */
  public TankStraight(DrivetrainSubsystem drive, DoubleSupplier left, DoubleSupplier right) {
    m_drive = drive;
    m_left = left;
    m_right = right;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    double l = m_left.getAsDouble();
    double r = m_right.getAsDouble();
    if (Math.abs(l-r) < Vars.DRIVE_THRESHOLD) {
      // average the inputs if the driver commands over the threshold
      l = (l+r)/2;
      // copy the value so both sides behave the same and go straight
      r = l;
    }
    m_drive.tankdriveSquared(l, r);
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
