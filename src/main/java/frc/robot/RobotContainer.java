/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.arm.ArmLinear;
import frc.robot.commands.arm.ArmStabilize;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.hopper.HopperGroupPower;
import frc.robot.commands.intake.IntakePower;
import frc.robot.commands.pit.ShooterCleaning;
import frc.robot.commands.shooter.ShooterRPM;
import frc.robot.commands.turret.TurretAim;
import frc.robot.commands.turret.TurretRotate;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // subsystems
  private final ArmSubsystem m_arm =  new ArmSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final CameraSubsystem m_camera = new CameraSubsystem();
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  private final HopperSubsystem m_hopper = new HopperSubsystem();
  private final FeedSubsystem m_feed = new FeedSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem();

  // commands
  private final ArmLinear m_armLinear = new ArmLinear(m_arm, ()->IO.getXBoxLeftX());
  // private final ArmUp m_armUp = new ArmUp(m_arm);

  private final TurretRotate m_rotate = new TurretRotate(m_turret, ()->IO.getXBoxRightX());
  private final TurretAim m_turretAim = new TurretAim(m_camera, m_turret);
  // private final TurretHome m_turretHome = new TurretHome(m_turret, 0);
  
  private final ShooterRPM m_shooterRPM = new ShooterRPM(m_shooter);
  private final ShooterCleaning m_shooterCleaning = new ShooterCleaning(m_shooter);

  private final IntakePower m_intakeBall = new IntakePower(m_intake, Vars.INTAKE_PERCENT);
  private final IntakePower m_intakeBall_Back = new IntakePower(m_intake, Vars.INTAKE_PERCENT_BACK);

  // private final DriveToDistance m_distance = new DriveToDistance(m_drivetrain, m_turret, m_camera, Vars.APPROACH_SETPOINT);
  private final TankDrive m_tankDrive = new TankDrive(m_drivetrain, ()->IO.getLeftY(), ()->IO.getRightY());

  private final HopperGroupPower m_hoppergroup = new HopperGroupPower(m_hopper, m_feed, Vars.HOPPER_WALL_PERCENT, Vars.HOPPER_FLOOR_PERCENT, Vars.FEED_PERCENT);
  private final HopperGroupPower m_hoppergroup_Back = new HopperGroupPower(m_hopper, m_feed, Vars.HOPPER_WALL_PERCENT_BACK, Vars.HOPPER_FLOOR_PERCENT_BACK, Vars.FEED_PERCENT_BACK);

  private final ArmToPosition m_armIn = new ArmToPosition(m_arm, Vars.ARM_IN);
  private final ArmToPosition m_armUp = new ArmToPosition(m_arm, Vars.ARM_UP);
  private final ArmToPosition m_armOut = new ArmToPosition(m_arm, Vars.ARM_OUT);
  private final SequentialCommandGroup m_armQuickZero = new InstantCommand(() -> {m_arm.stop();m_arm.reset();}).andThen(new ArmStabilize(m_arm));
  
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
    
    IO.leftJoystick_4.whenPressed(m_armOut);
    IO.rightJoystick_1.whileHeld(m_turretAim);
    IO.rightJoystick_2.whileHeld(m_shooterRPM);
    // IO.xbox_A.whileHeld(m_armLinear);
    IO.xbox_B.whenPressed(m_armIn);
    IO.xbox_Y.whenPressed(m_armUp);
    IO.xbox_LB.whileHeld(m_intakeBall_Back);
    IO.xbox_RB.whileHeld(m_hoppergroup_Back);
    IO.xbox_LT.whileHeld(m_intakeBall);
    IO.xbox_RT.whileHeld(m_hoppergroup);
    
    IO.xbox_R_JOYSTICK.whileHeld(m_rotate);
    IO.leftJoystick_8.whileHeld(m_shooterCleaning);
    IO.leftJoystick_9.whenPressed(m_armQuickZero);

  }

  /**
   * Runs once at the start of teleop
   */
  public void startup() {
    CommandScheduler.getInstance().schedule(new ArmStabilize(m_arm));
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
