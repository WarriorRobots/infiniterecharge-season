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
 * A 90 degree turn at one point.
 * Start at the origin and just turn 90 degrees.
 */
public class TPoint90 extends TBase {

  public TPoint90(boolean left) {
    super(left);
  }

  @Override
  double maxSpeed() {
    return Units.feetToMeters(5);
  }

  @Override
  double maxAcceleration() {
    return Units.feetToMeters(5);
  }

  @Override
  void build() {
    start = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(0));
    end = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(90 * (LEFT ? 1 : -1)));
  }

}
