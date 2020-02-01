/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.trajectories;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Vars;

/**
 * Base trajectory class for 2478.
 * Streamlines the process of making a {@link Trajectory} using the WPI tools.
 * <p>
 * Requires at least a {@link #build()} function to set the {@link #start}, {@link #end},
 * and {@link #Waypoints}.
 * 
 * @author Joshua B.
 */
public abstract class TBase {

  /** Max speed of trajectory */
  final double MAX_SPEED = maxSpeed();
  /** Max acceleration of trajectory (in m/s) */
  final double MAX_ACCELERATION = maxAcceleration();
  /** Starting velocity of the robot along the trajectory in m/s */
  final double START_VELOCITY = startSpeed();
  /** Ending velocity of the robot along the trajectory in m/s */
  final double END_VELOCITY = endSpeed();
  /** Whether the robot drives backwards along the path */
  final boolean REVERSED = isReversed();

  /** Whether the trajectory is the left or right mirror of the trajectory. */
  final boolean LEFT;

  /** Pose of the robot at the start of the trajectory. */
  Pose2d start;
  /** Pose of the robot at the end of the trajectory. */
  Pose2d end;
  /** All interior waypoints the robot must pass through on the trajectory. */
  ArrayList<Translation2d> Waypoints = new ArrayList<Translation2d>();
  /** Configuration of the trajectory, involves the speed, acceleration, and direction of the trajectory. */
  TrajectoryConfig config;
  /** Constraint for the robot to obey when following the trajectory */
  TrajectoryConstraint constraint;

  /** Final trajectory produced by the class. */
  Trajectory trajectory;

  /**
   * Returns the max speed the robot travels on the trajectory.
   * <p>
   * If not overrided, returns the max speed of the robot.
   * 
   * @return max speed of the robot in m/s.
   */
  double maxSpeed() {
    return Units.inchesToMeters(Vars.MAX_VELOCITY);
  }

  /**
   * Returns the max acceleration the robot travels on the trajectory.
   * <p>
   * If not overrided, returns the max acceleration of the robot.
   * 
   * @return max acceleration of the robot in m/s^2.
   */
  double maxAcceleration() {
    return Units.inchesToMeters(Vars.MAX_ACCELERATION);
  }
  
  /**
   * Returns the start speed of the robot.
   * If not overrided, returns 0.
   * 
   * @return The speed of the robot at the start of the trajectory in m/s.
   */
  public double startSpeed() {
    return 0;
  }

  /**
   * Returns the end speed of the robot
   * If not overrided, returns 0.
   * 
   * @return The speed of the robot at the start of the trajectory in m/s.
   */
  public double endSpeed() {
    return 0;
  }

  /**
   * Returns if the robot travels backwards on the trajectory. Traveling backwards
   * means the robot still goes through the points in order, but in the reversed
   * order. Travelling reversed also means the heading of the poses are still the
   * forwards direction of the robot.
   * <p>
   * If not overrided, returns false.
   * 
   * @return whether the robot travels backwards on the trajectory.
   */
  public boolean isReversed() {
    return false;
  }

  /**
   * Sets the start pose, end pose, and any waypoints inbetween.
   * @see #start
   * @see #end
   * @see #Waypoints
   */
  abstract void build();

  /**
   * Constructs trajectory from given start, end, and waypoints from {@link build()}.
   */
  public TBase() {
    this(true);
  }

  /**
   * Constructs trajectory from given start, end, and waypoints from {@link build()}.
   * <p>
   * NOTE: when trying to use this, you must create a constructor to call this constructor:
   * <pre> public TName(boolean left) {super(left);} </pre>
   * @param left Whether the trajectory is the left or right mirror of the trajectory
   */
  public TBase(boolean left) {
    LEFT = left;
    build();

    // constraint = new DifferentialDriveKinematicsConstraint(Vars.kDriveKinematics, MAX_SPEED);
    constraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Vars.ksVolts, Vars.kvVoltSecondsPerMeter, Vars.kaVoltSecondsSquaredPerMeter),
      Vars.kDriveKinematics, 
      10
    );

    config = new TrajectoryConfig(
      Units.inchesToMeters(MAX_SPEED),
      Units.inchesToMeters(MAX_ACCELERATION)
    );
    config.setReversed(REVERSED);
    config.setKinematics(Vars.kDriveKinematics);
    config.addConstraint(constraint);
    config.setStartVelocity(START_VELOCITY);
    config.setEndVelocity(END_VELOCITY);

    trajectory = TrajectoryGenerator.generateTrajectory(start, Waypoints, end, config);
  }

  /**
   * @return Trajectory build by the constructor.
   */
  public final Trajectory getTrajectory() {
    return trajectory;
  }

}
