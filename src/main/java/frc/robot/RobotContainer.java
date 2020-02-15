/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.drive.DriveToDistance;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.shooter.ShooterRPM;
import frc.robot.commands.turret.TurretAim;
import frc.robot.commands.turret.TurretRotate;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.KitDriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CameraSubsystem m_camera = new CameraSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem();
  private final KitDriveSubsystem m_drive = new KitDriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();


  private final TurretRotate m_rotate = new TurretRotate(m_turret, () -> IO.getXBoxRightX());
  private final TurretAim m_turretAim = new TurretAim(m_camera, m_turret);
  private final ShooterRPM m_shooterRPM = new ShooterRPM(m_shooter);

  private final DriveToDistance m_distance = new DriveToDistance(m_drive, m_turret, m_camera, Vars.APPROACH_SETPOINT);

  private final TankDrive m_tankDrive = new TankDrive(m_drive, ()->IO.getLeftY(), ()->IO.getRightY());

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    CommandScheduler.getInstance().setDefaultCommand(m_drive, m_tankDrive);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    IO.left1.whileHeld(m_distance);
    IO.right1.whileHeld(m_turretAim);
    IO.right2.whileHeld(m_shooterRPM);
    IO.xbox_RB.whileHeld(m_rotate);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
