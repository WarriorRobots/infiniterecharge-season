/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.climb.ClimbLock;
import frc.robot.commands.climb.ClimbToPosition;
import frc.robot.commands.climb.ClimbUnlock;
import frc.robot.commands.climb.LinearClimb;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.hopper.HopperPower;
import frc.robot.commands.pit.ShooterCleaning;
import frc.robot.commands.shooter.ShooterRPM;
import frc.robot.commands.shooter.ShooterVoltage;
import frc.robot.commands.turret.TurretAim;
import frc.robot.commands.turret.TurretHome;
import frc.robot.commands.turret.TurretRotate;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
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
  
  // subsystems
  // private final ArmSubsystem m_arm =  new ArmSubsystem();
  private final CameraSubsystem m_camera = new CameraSubsystem();
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  private final HopperSubsystem m_hopper = new HopperSubsystem();
  // private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem();
  private final ClimbSubsystem m_climb = new ClimbSubsystem();

  // commands
  // private final ArmRotate m_armRotate = new ArmRotate(m_arm, ()->IO.getLeftX());
  // private final ArmIntake m_armIntake = new ArmIntake(m_arm);
  // private final ArmUp m_armUp = new ArmUp(m_arm);

  private final TurretRotate m_rotate = new TurretRotate(m_turret, ()->IO.getXBoxRightX());
  private final TurretAim m_turretAim = new TurretAim(m_camera, m_turret);
  private final TurretHome m_turretHome = new TurretHome(m_turret, 0);
  
  private final ShooterRPM m_shooterRPM = new ShooterRPM(m_shooter);
  private final ShooterVoltage m_shooterVoltage = new ShooterVoltage(m_shooter);
  private final ShooterCleaning m_shooterCleaning = new ShooterCleaning(m_shooter);

  // private final DriveToDistance m_distance = new DriveToDistance(m_drivetrain, m_turret, m_camera, Vars.APPROACH_SETPOINT);
  private final TankDrive m_tankDrive = new TankDrive(m_drivetrain, ()->IO.getLeftY(), ()->IO.getRightY());

  private final HopperPower m_hopperPower = new HopperPower(m_hopper);

  private final ClimbToPosition m_climbUpPosition = new ClimbToPosition(m_climb, Vars.CLIMB_UP_POSITION);
  private final ClimbToPosition m_climbUpDown = new ClimbToPosition(m_climb, Vars.CLIMB_DOWN_POSITION);
  private final LinearClimb m_linearClimb = new LinearClimb(m_climb, ()->IO.getLeftY()); // TODO change this
  private final ClimbLock m_climbLock = new ClimbLock(m_climb);
  private final ClimbUnlock m_climbUnlock = new ClimbUnlock(m_climb);



  // private final FeedBall m_feedBall = new FeedBall(m_intake, ()->IO.getLeftSlider());
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_tankDrive);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    IO.leftJoystick_8.whileHeld(m_shooterCleaning);
    // IO.rightJoystick_8.whileHeld(m_turretHome);
    IO.rightJoystick_2.whileHeld(m_hopperPower);
    IO.xbox_B.whileHeld(m_turretAim);
    IO.xbox_RB.whileHeld(m_shooterRPM);
    IO.xbox_LB.whileHeld(m_shooterVoltage);
    IO.xbox_RT.whileHeld(m_rotate);

    //IO.leftJoystick_11.whileHeld(m_armRotate);
    // IO.leftJoystick_12.whileHeld(m_armUp);
    // IO.rightJoystick_3.whileHeld(m_armIntake);

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
