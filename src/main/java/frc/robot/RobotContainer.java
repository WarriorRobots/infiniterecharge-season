/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.commands.TankDrive;
import frc.robot.commands.drive.DriveOdomReset;
import frc.robot.commands.drive.RamseteContainer;
import frc.robot.commands.drive.trajectories.TBack;
import frc.robot.commands.drive.trajectories.TCircle90;
import frc.robot.commands.drive.trajectories.TPoint90;
import frc.robot.commands.drive.trajectories.TSaxonTurn;
import frc.robot.commands.drive.trajectories.TStraight;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();

  private final TankDrive m_tankCommand = new TankDrive(m_drivetrain);
  private final DriveOdomReset m_drivereset = new DriveOdomReset(m_drivetrain);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_tankCommand);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    IO.left12.whenPressed(m_drivereset);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new RamseteContainer(m_drivetrain, new TStraight(), new TStraight()).getCommand();

    // // Create a voltage constraint to ensure we don't accelerate too fast
    // var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(Vars.ksVolts,
    //                                    Vars.kvVoltSecondsPerMeter,
    //                                    Vars.kaVoltSecondsSquaredPerMeter),
    //         Vars.kDriveKinematics,
    //         10);

    // // Create config for trajectory
    // TrajectoryConfig config =
    //     new TrajectoryConfig(Units.inchesToMeters(100),
    //                          Units.inchesToMeters(100))
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(Vars.kDriveKinematics)
    //         // Apply the voltage constraint
    //         .addConstraint(autoVoltageConstraint);

    // // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(
    //         // new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(40)),
    //         // new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(-40))
    //     ),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(Units.inchesToMeters(60), 0, new Rotation2d(0)),
    //     // Pass config
    //     config
    // );

    // // Paste this variable in
    // RamseteController disabledRamsete = new RamseteController() {
    //   @Override
    //   public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
    //           double angularVelocityRefRadiansPerSecond) {
    //       return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
    //   }
    // };

    // var leftController = new PIDController(Vars.kPDriveVel, 0, 0);
    // var rightController = new PIDController(Vars.kPDriveVel, 0, 0);
    // RamseteCommand ramseteCommand = new RamseteCommand(
    //     exampleTrajectory,
    //     // new TStraight().getTrajectory(),
    //     m_drivetrain::getPose,
    //     new RamseteController(Vars.kRamseteB, Vars.kRamseteZeta),
    //     // disabledRamsete,
    //     new SimpleMotorFeedforward(Vars.ksVolts,
    //                                Vars.kvVoltSecondsPerMeter,
    //                                Vars.kaVoltSecondsSquaredPerMeter),
    //     Vars.kDriveKinematics,
    //     m_drivetrain::getWheelSpeeds,
    //     leftController,
    //     rightController,
    //     // RamseteCommand passes volts to the callback
    //     (l,r)->{
    //       m_drivetrain.tankdriveVoltage(l, r);
    //       SmartDashboard.putNumber("left_measurement", m_drivetrain.getWheelSpeeds().leftMetersPerSecond);
    //       SmartDashboard.putNumber("left_reference", leftController.getSetpoint());
    //       SmartDashboard.putNumber("right_measurement", m_drivetrain.getWheelSpeeds().rightMetersPerSecond);
    //       SmartDashboard.putNumber("right_reference", rightController.getSetpoint());
    //     },
    //     m_drivetrain
    // );

    // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> m_drivetrain.tankdriveVoltage(0, 0));
    // return new RamseteContainer(m_drivetrain, exampleTrajectory).getCommand();
  }
}
