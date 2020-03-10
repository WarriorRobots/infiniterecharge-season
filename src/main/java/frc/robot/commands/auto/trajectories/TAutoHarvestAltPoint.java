/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.trajectories;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Vars;

/**
 * A path from the trench to the center.
 */
public class TAutoHarvestAltPoint extends TBase{

  @Override
  void build() {
    start = new Pose2d();
    Waypoints.add(new Translation2d(Units.inchesToMeters(Vars.HARVESTALT_POINT_1_X), Units.inchesToMeters(Vars.HARVESTALT_POINT_1_Y)));
    end = new Pose2d(Units.inchesToMeters(Vars.HARVESTALT_POINT_2_X), Units.inchesToMeters(Vars.HARVESTALT_POINT_2_Y), new Rotation2d(Vars.HARVESTALT_POINT_2_T));
  }

}
