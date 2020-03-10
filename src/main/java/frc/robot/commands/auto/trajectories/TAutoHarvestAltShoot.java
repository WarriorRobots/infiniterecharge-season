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
import frc.robot.Vars;

/**
 * A path from the center to the shoot location.
 */
public class TAutoHarvestAltShoot extends TBase {

  @Override
  void build() {
    start = new Pose2d();
    end = new Pose2d(Units.inchesToMeters(Vars.HARVESTALT_BACK_X), Units.inchesToMeters(Vars.HARVESTALT_BACK_Y), new Rotation2d(Vars.HARVESTALT_BACK_T));
  }
}
