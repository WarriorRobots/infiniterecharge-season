/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.trajectories;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;

/**
 * A straight path, but backwards.
 * Start at the origin and move 5 feet backwards in the x direction.
 */
public class TBack extends TBase {

  @Override
  boolean isReversed() {
    return true;
  }

  @Override
  void build() {
    start = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(0));
    end = new Pose2d(Units.feetToMeters(-5), Units.feetToMeters(0), Rotation2d.fromDegrees(0));
  }

}
