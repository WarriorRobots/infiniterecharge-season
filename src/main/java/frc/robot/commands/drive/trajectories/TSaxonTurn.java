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
 * Go forwards 3 feet, make a turn similar to TCircle90, and go another 3 feet.
 */
public class TSaxonTurn extends TBase {

    public TSaxonTurn(boolean left) {
        super(left);
    }

    @Override
    void build() {
    start = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(0));
    // 3 feet forwards
    Waypoints.add(new Translation2d(Units.feetToMeters(3), Units.feetToMeters(0)));
    // on the turn
    // (5/sqrt(2) + 3, 5*(1-1/sqrt(2))) ~ (6.525, 1.464)
    Waypoints.add(new Translation2d(Units.feetToMeters(6.525), Units.feetToMeters(1.464 * (LEFT ? 1 : -1))));
    // after the turn
    Waypoints.add(new Translation2d(Units.feetToMeters(8), Units.feetToMeters(5 * (LEFT ? 1 : -1))));
    end = new Pose2d(Units.feetToMeters(8), Units.feetToMeters(8 * (LEFT ? 1 : -1)), Rotation2d.fromDegrees(90 * (LEFT ? 1 : -1)));
    }
}
