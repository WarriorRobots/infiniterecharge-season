/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.DashboardContainer.TabsIndex;
import frc.robot.commands.arm.ArmHoldPosition;
import frc.robot.commands.arm.ArmLinear;
import frc.robot.commands.arm.ArmStabilize;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.arm.ArmZero;
import frc.robot.commands.camera.CameraChangePipeline;
import frc.robot.commands.climb.ClimbBrakes;
import frc.robot.commands.climb.ClimbLinear;
import frc.robot.commands.climb.ClimbToPosition;
import frc.robot.commands.drive.TankStation;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.drive.TankStraight;
import frc.robot.commands.hopper.HopperGroupPower;
import frc.robot.commands.intake.IntakeHopper;
import frc.robot.commands.intake.IntakePower;
import frc.robot.commands.pit.ShooterCleaning;
import frc.robot.commands.shooter.ShooterHopper;
import frc.robot.commands.shooter.ShooterPrep;
import frc.robot.commands.shooter.ShooterRPM;
// import frc.robot.commands.shooter.ShooterSequence;
import frc.robot.commands.turret.TurretAimSequence;
import frc.robot.commands.turret.TurretPreset;
import frc.robot.commands.turret.TurretRotate;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ClimbSubsystem.Brakes;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // subsystems
  protected static final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  protected static final ShooterSubsystem m_shooter = new ShooterSubsystem();
  protected static final TurretSubsystem m_turret = new TurretSubsystem();
  protected static final CameraSubsystem m_camera = new CameraSubsystem();
  protected static final FeedSubsystem m_feed = new FeedSubsystem();
  protected static final HopperSubsystem m_hopper = new HopperSubsystem();
  protected static final ArmSubsystem m_arm =  new ArmSubsystem();
  protected static final IntakeSubsystem m_intake = new IntakeSubsystem();
  protected static final ClimbSubsystem m_climb = new ClimbSubsystem();

  // commands
  private final TankDrive m_tankDrive = new TankDrive(m_drivetrain, ()->IO.getLeftY(), ()->IO.getRightY());
  private final TankStraight m_tankDriveStraight = new TankStraight(m_drivetrain, ()->IO.getLeftY(), ()->IO.getRightY());
  private final TankStation m_tankStation = new TankStation(m_drivetrain, m_turret, m_camera, ()->IO.getLeftY(), ()->IO.getRightY());
  
  // private final ShooterRPM m_shooterRPM = new ShooterRPM(m_shooter);
  private final SequentialCommandGroup m_shooterSequence = new SequentialCommandGroup(
    new ShooterPrep(m_hopper, m_feed),
    new ShooterHopper(m_shooter, m_intake, m_hopper, m_feed){public void end(boolean interrupted){/* This is empty is to not stop the motor from rev-ing*/}}
  ){public void end(boolean interrupted){m_intake.stop();m_hopper.stop();m_feed.stop();}}; // This is to stop the hopper and feed if the command is stopped however not stop the shooter, that is handled by UnRev
  /** Clears the shooter and runs the shooter at an rpm for the shooter to then be fed */
  private final SequentialCommandGroup m_revTrigger = new SequentialCommandGroup(
    new ShooterPrep(m_hopper, m_feed),
    new ShooterRPM(m_shooter){public void end(boolean interrupted){/* This is empty is to not stop the motor from rev-ing*/}}
  );
  /** Stop the shooter */
  private final InstantCommand m_shooterUnRev = new InstantCommand(() -> m_shooter.stop(), m_shooter);
  private final ShooterCleaning m_shooterCleaning = new ShooterCleaning(m_shooter);

  private final TurretRotate m_turretRotate = new TurretRotate(m_turret, ()->IO.getXBoxRightX());
  private final TurretAimSequence m_turretAim = new TurretAimSequence(m_camera, m_turret, m_arm, false);
  private final TurretPreset m_turretForwards = new TurretPreset(m_turret, 0);
  private final TurretPreset m_turretBackwards = new TurretPreset(m_turret, -180); // -180 because the turret turns left
  private final InstantCommand m_turretQuickZero = new InstantCommand(() -> m_turret.resetEncoder()){public boolean runsWhenDisabled(){return true;}};

  private final CameraChangePipeline m_cameraDriver = new CameraChangePipeline(m_camera, CameraSubsystem.PIPELINE_DRIVER){public boolean runsWhenDisabled(){return true;}};
  private final CameraChangePipeline m_cameraHex = new CameraChangePipeline(m_camera, CameraSubsystem.PIPELINE_HEX){public boolean runsWhenDisabled(){return true;}};

  private final HopperGroupPower m_hoppergroup = new HopperGroupPower(m_hopper, m_feed, Vars.HOPPER_WALL_PERCENT, Vars.HOPPER_FLOOR_PERCENT, Vars.FEED_PERCENT);
  private final HopperGroupPower m_hoppergroup_Back = new HopperGroupPower(m_hopper, m_feed, Vars.HOPPER_WALL_PERCENT_BACK, Vars.HOPPER_FLOOR_PERCENT_BACK, Vars.FEED_PERCENT_BACK);

  private final ArmLinear m_armLinear = new ArmLinear(m_arm, ()->IO.getXBoxLeftY());
  // private final ArmUp m_armUp = new ArmUp(m_arm);

  // private final IntakePower m_intakeBall = new IntakePower(m_intake, Vars.INTAKE_PERCENT);
  private final IntakeHopper m_intakeBall = new IntakeHopper(m_intake, m_hopper, m_feed);
  private final IntakePower m_intakeBall_Back = new IntakePower(m_intake, Vars.INTAKE_PERCENT_BACK);

  private final ArmToPosition m_armIn = new ArmToPosition(m_arm, Vars.ARM_IN);
  private final ArmToPosition m_armPlayer = new ArmToPosition(m_arm, Vars.ARM_PLAYER);
  // private final ArmHoldPosition m_armOut = new ArmHoldPosition(m_arm, Vars.ARM_OUT); // note this is a hold button
  private final ParallelCommandGroup m_pickupSequence = new ParallelCommandGroup(
    new ArmHoldPosition(m_arm, Vars.ARM_OUT),
    new IntakeHopper(m_intake, m_hopper, m_feed)
  );
  private final ArmZero m_armZero = new ArmZero(m_arm);

  private final ClimbLinear m_climbLinear = new ClimbLinear(m_climb, ()->IO.getXBoxLeftY()); // XXX change binding or remove binding
  // private final ClimbToPosition m_climbUp = new ClimbToPosition(m_climb, Vars.CLIMB_UP);
  // private final ClimbToPosition m_climbDown = new ClimbToPosition(m_climb, Vars.CLIMB_DOWN);
  private final ClimbBrakes m_engageBrake = new ClimbBrakes(m_climb, Brakes.engage);

  // private final AutoLinear m_autoTestForwards = new AutoLinear(m_drivetrain, 20);
  // private final AutoAngular m_autoTestRight = new AutoAngular(m_drivetrain, 90);
  // private final SequentialCommandGroup m_autoTestSquare = new SequentialCommandGroup(
  //   new AutoLinear(m_drivetrain, 20),
  //   new AutoAngular(m_drivetrain, 90),
  //   new AutoLinear(m_drivetrain, 20),
  //   new AutoAngular(m_drivetrain, 90),
  //   new AutoLinear(m_drivetrain, 20),
  //   new AutoAngular(m_drivetrain, 90),
  //   new AutoLinear(m_drivetrain, 20),
  //   new AutoAngular(m_drivetrain, 90)
  // );
  // private final AutoHarvest m_autoHarvest = new AutoHarvest(m_drivetrain, m_shooter, m_turret, m_camera, m_feed, m_hopper, m_arm, m_intake);
  // private final InstantCommand m_driveReset = new InstantCommand(() -> m_drivetrain.resetOdometry(), m_drivetrain){public boolean runsWhenDisabled(){return true;}};

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
    
    IO.leftJoystick_1.whileHeld(m_tankStation);
    IO.leftJoystick_4.whileHeld(m_tankDriveStraight);
    IO.rightJoystick_1.whileHeld(m_turretAim).whileActiveOnce(m_revTrigger, true);
    IO.rightJoystick_2.whileHeld(m_shooterSequence);
    IO.rightJoystick_1.negate().and(IO.rightJoystick_2.negate()).whileActiveOnce(m_shooterUnRev); // This is for if the driver revs the shooter but does not shoot so the shooter stops
    IO.rightJoystick_3.whileHeld(m_pickupSequence);
    // IO.rightJoystick_6.whenPressed(m_climbDown);
    IO.xbox_B.whenPressed(m_armPlayer);
    IO.xbox_Y.whenPressed(m_armIn);
    IO.xbox_LB.whileHeld(m_intakeBall_Back);
    IO.xbox_LT.whileHeld(m_intakeBall);
    IO.xbox_RT.whileHeld(m_climbLinear).whenReleased(m_engageBrake);
    // IO.xbox_START.whenPressed(m_climbUp);
    // xbox select and start may be for climb
    IO.xboxUp.whenPressed(m_turretForwards);
    IO.xboxDown.whenPressed(m_turretBackwards);
    IO.xbox_R_UP.and(IO.xbox_R_JOYSTICK.negate()).whileActiveOnce(m_hoppergroup, true); // when the stick is not pressed and pushed up
    IO.xbox_R_DOWN.and(IO.xbox_R_JOYSTICK.negate()).whileActiveOnce(m_hoppergroup_Back, true); // when the stick is not pressed and pushed down
    
    // debug
    IO.xbox_L_JOYSTICK.whileHeld(m_armLinear);
    IO.xbox_R_JOYSTICK.whileHeld(m_turretRotate);
    IO.leftJoystick_7.whenPressed(m_armZero);
    IO.leftJoystick_8.whileHeld(m_shooterCleaning);
    IO.leftJoystick_9.whenPressed(m_turretQuickZero);
    IO.rightJoystick_7.whenPressed(m_cameraDriver);
    IO.rightJoystick_8.whenPressed(m_cameraHex);

  }

  /**
   * Runs once at the start of teleop
   * @param enable Set to true if the robot was just enabled
   * (there are scenarios where this could be false, eg moving from auto to teleop)
   */
  public void startup(boolean enable) {

    if (enable) {
      // run the commands that only occur when the enable button was just pressed
      new ArmStabilize(m_arm).schedule();

      DashboardContainer.getInstance().setTab(TabsIndex.kDriver);
    }

    // run the commands for startup

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new RamseteContainer(m_drivetrain, new TLine(){public double getLengthIn(){return 84;}}).getCommandAndStop();
    // return m_autoHarvest;
    // return new RamseteContainer(m_drivetrain, new TWPI()).getCommandAndStop();
    return AutoContainer.getInstance().getAutoCommand();
  }

  /**
   * Stops all the devices on the robot.
   * Used when the robot disables to not allow any processes to keep running.
   * (E.G. if the auto is stopped and the shooter is not told to stop, it is told to stop here.)
   * @return
   */
  public Command getStopAll() {
    return new InstantCommand( () -> {
        m_drivetrain.stop();
        m_shooter.stop();
        m_shooter.stop();
        m_feed.stop();
        m_hopper.stop();
        m_arm.stop();
        m_intake.stop();
        m_climb.stopWinch();
      }
    ) {public boolean runsWhenDisabled(){return true;}};
  }
}
