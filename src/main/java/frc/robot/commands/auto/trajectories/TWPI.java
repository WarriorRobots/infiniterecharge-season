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

/**
 * Add your docs here.
 */
public class TWPI extends TBase {

    @Override
    void build() {
        // All units are in meters
        
        // Start at the origin facing the +X direction
        start = new Pose2d(0, 0, new Rotation2d(0));
        // Pass through these two interior waypoints, making an 's' curve path
        Waypoints.add(new Translation2d(1, 1));
        Waypoints.add(new Translation2d(2, -1));
        // End 3 meters straight ahead of where we started, facing forward
        end = new Pose2d(3, 0, new Rotation2d(0));
    }

}
