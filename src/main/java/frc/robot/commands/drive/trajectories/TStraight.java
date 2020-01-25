/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.trajectories;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;

/**
 * Add your docs here.
 */
public class TStraight{

  /** Max speed of trajectory */
  private static final double MAX_SPEED = Units.feetToMeters(5);
  /** Max acceleration of trajectory (in m/s) */
  private static final double MAX_ACCELERATION = Units.feetToMeters(5);
  /** Whether the robot drives backwards along the path */
  private static final boolean REVERSED = false;

  private Pose2d start;
  private Pose2d end;
  private ArrayList<Translation2d> Waypoints = new ArrayList<Translation2d>();
  private TrajectoryConfig config;

  private Trajectory trajectory;

  public TStraight() {
    start = new Pose2d(Units.feetToMeters(0),Units.feetToMeters(0),Rotation2d.fromDegrees(0));
    end = new Pose2d(Units.feetToMeters(5),Units.feetToMeters(0),Rotation2d.fromDegrees(0));

    config = new TrajectoryConfig(MAX_SPEED, MAX_ACCELERATION);
    config.setReversed(REVERSED);

    trajectory = TrajectoryGenerator.generateTrajectory(start, Waypoints, end, config);
  }

  public Trajectory getTrajectory() {
    return trajectory;
  }

}
