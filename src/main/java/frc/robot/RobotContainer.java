/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.arm.ArmIntake;
import frc.robot.commands.arm.ArmUp;
import frc.robot.commands.camera.CameraChangePipeline;
import frc.robot.commands.camera.GetOffsetAngle;
import frc.robot.commands.arm.ArmRotate;
import frc.robot.commands.hopper.HopperPower;
import frc.robot.commands.intake.FeedBall;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.drive.DriveToDistance;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.shooter.AimANDFire;
import frc.robot.commands.shooter.ShooterRPM;
import frc.robot.commands.shooter.ShooterVoltage;
import frc.robot.commands.turret.TurretAim;
import frc.robot.commands.turret.TurretRotate;
import frc.robot.commands.turret.TurretHome;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.KitDriveSubsystem;
import frc.robot.subsystems.LedControllerSubsystem;
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
  // subsystems
  private final ArmSubsystem m_monkey =  new ArmSubsystem();
  private final CameraSubsystem m_camera = new CameraSubsystem();
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  private final HopperSubsystem m_hippityhop = new HopperSubsystem();
  private final IntakeSubsystem m_hungryhippo = new IntakeSubsystem();
  private final KitDriveSubsystem m_drive = new KitDriveSubsystem();
  private final LedControllerSubsystem m_blink = new LedControllerSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem();

  // commands
  private final ArmRotate m_armRotate = new ArmRotate(m_monkey, ()->IO.getLeftX());
  private final ArmIntake m_armIntake = new ArmIntake(m_monkey);
  private final ArmUp m_armUp = new ArmUp(m_monkey);

  // CameraChangePipeline command here
  private final GetOffsetAngle m_offsetAngle = new GetOffsetAngle(m_camera, m_shooter, m_turret);

  private final DriveToDistance m_distance = new DriveToDistance(m_drive, m_turret, m_camera, Vars.APPROACH_SETPOINT);
  private final TankDrive m_tankDrive = new TankDrive(m_drive, ()->IO.getLeftZ(), ()->IO.getRightZ());

  private final HopperPower m_hopperPower = new HopperPower(m_hippityhop, ()->IO.getRightSlider(), ()->IO.getRightX());

  private final FeedBall m_feedBall = new FeedBall(m_hungryhippo, ()->IO.getLeftSlider());

  private final AimANDFire m_aimANDfire = new AimANDFire(m_shooter, m_turret, m_camera);
  private final ShooterRPM m_shooterRPM = new ShooterRPM(m_shooter);
  private final ShooterVoltage m_shooterVoltage = new ShooterVoltage(m_shooter, ()->IO.getXBoxRightY()); // TODO change this later?

  private final TurretAim m_turretAim = new TurretAim(m_camera, m_turret);
  private final TurretRotate m_rotate = new TurretRotate(m_turret, () -> IO.getXBoxRightX());
  private final TurretHome m_turretHome = new TurretHome(m_turret, 0);
 
  // TODO put these on different joysticks uh yeah -Joshua
  
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
    IO.leftJoystick_1.whileHeld(m_distance);
    IO.leftJoystick_2.whileHeld(m_feedBall);
    IO.leftJoystick_3.whileHeld(m_aimANDfire);
    IO.leftJoystick_4.whileHeld(m_shooterRPM);
    //IO.leftJoystick_5.whileHeld(command);
    //IO.leftJoystick_6.whileHeld(command);
    //IO.leftJoystick_7.whileHeld(command);
    //IO.leftJoystick_8.whileHeld(command);
    //IO.leftJoystick_9.whileHeld(command);
    //IO.leftJoystick_10.whileHeld(command);
    //IO.leftJoystick_11.whileHeld(m_armRotate);
    IO.leftJoystick_12.whileHeld(m_armUp);

    IO.rightJoystick_1.whileHeld(m_turretAim);
    IO.rightJoystick_2.whileHeld(m_shooterRPM);
    IO.rightJoystick_3.whileHeld(m_armIntake);
    IO.rightJoystick_4.whileHeld(m_tankDrive);
    //IO.rightJoystick_5.whileHeld(command);
    //IO.rightJoystick_6.whileHeld(command);
    //IO.rightJoystick_7.whileHeld(command);
    //IO.rightJoystick_8.whileHeld(command);
    IO.rightJoystick_9.whileHeld(m_turretAim);
    IO.rightJoystick_10.whileHeld(m_turretHome);
    //IO.rightJoystick_11.whileHeld(command);
    IO.rightJoystick_12.whileHeld(m_hopperPower);
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
