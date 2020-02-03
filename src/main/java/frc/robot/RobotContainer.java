/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.TankDrive;
import frc.robot.commands.camera.CameraChangePipeline;
import frc.robot.commands.camera.CameraStopAtDistance;
import frc.robot.commands.led.LEDChangePattern;
import frc.robot.commands.shooter.AimANDFire;
import frc.robot.commands.shooter.ShooterRPM;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LedControllerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  private final ShooterSubsystem m_pewpew = new ShooterSubsystem();
  private final CameraSubsystem m_snapsnap = new CameraSubsystem();
  private final LedControllerSubsystem m_scan = new LedControllerSubsystem();

  // drivetrain commands
  private final TankDrive m_tankCommand = new TankDrive(m_drivetrain);

  // shooter commands
  private final AimANDFire m_aimFire = new AimANDFire(m_pewpew, m_drivetrain, m_snapsnap);
  private final ShooterRPM m_rpm = new ShooterRPM(m_pewpew);
  
  // camera commands
  private final CameraStopAtDistance m_cameraStop = new CameraStopAtDistance(m_drivetrain, m_snapsnap, m_scan,
    /** finishable */);

  private final CameraChangePipeline m_changePipe = new CameraChangePipeline(m_snapsnap, /** pipeline */);
  // could add private pipeline variable to fix

  // led commands
  private final LEDChangePattern m_LCP = new LEDChangePattern(/** channel */, m_scan); // don't know what value you wanted

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
