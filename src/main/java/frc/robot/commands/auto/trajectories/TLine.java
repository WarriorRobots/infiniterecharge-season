/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.trajectories;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;

/**
 * An abstract line trajectory that has a length.
 */
public abstract class TLine extends TBase {

  /** Length of the line trajectory in inches */
  public abstract double getLengthIn();

  @Override
  void build() {
    start = new Pose2d();
    end = new Pose2d(Units.inchesToMeters(getLengthIn()), 0, new Rotation2d());
  }
}
