/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.trajectories;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

/**
 * A 90 degree turn in the shape of a quarter circle.
 * Start at the origin and travel left or right around a 5 foot radius circle.
 */
public class TCircle90 extends TBase {

  public TCircle90(boolean left) {
    super(left);
  }

  @Override
  void build() {
    start = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(0));
    Waypoints.add(new Translation2d(Units.feetToMeters(5/Math.sqrt(2)), Units.feetToMeters(5/Math.sqrt(2) * (LEFT ? 1 : -1))));
    end = new Pose2d(Units.feetToMeters(5), Units.feetToMeters(5 * (LEFT ? 1 : -1)), Rotation2d.fromDegrees(90 * (LEFT ? 1 : -1)));
  }

}
