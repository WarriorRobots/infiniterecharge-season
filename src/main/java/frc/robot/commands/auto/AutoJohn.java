/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Vars;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.arm.ArmZero;
import frc.robot.commands.auto.trajectories.TBase;
import frc.robot.commands.auto.trajectories.TLine;
import frc.robot.commands.camera.CameraChangePipeline;
import frc.robot.commands.drive.AutoLinear;
import frc.robot.commands.feed.FeedBall;
import frc.robot.commands.hopper.HopperPower;
import frc.robot.commands.intake.IntakeHopper;
import frc.robot.commands.intake.IntakePower;
import frc.robot.commands.shooter.ShooterHopper;
import frc.robot.commands.shooter.ShooterPrep;
import frc.robot.commands.shooter.ShooterRPM;
import frc.robot.commands.shooter.ShooterSequence;
import frc.robot.commands.turret.TurretAim;
import frc.robot.commands.turret.TurretPreset;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
@Deprecated
public class AutoJohn extends SequentialCommandGroup {
  /**
   * Robot faces forwards towards the trench (84") and turns to face the target (~ -220),
   * shoots the balls for ~ 3 seconds,
   * drives forwards (115") and (put the arm down then runs the intake the whole time),
   * go backwards to the back of the trench again (115")
   * shoots the balls for ~ 5
   */
  public AutoJohn(
                      DrivetrainSubsystem m_drivetrain,
                      TurretSubsystem turret,
                      CameraSubsystem camera,
                      ShooterSubsystem shooter,
                      HopperSubsystem hopper,
                      FeedSubsystem feed,
                      ArmSubsystem arm,
                      IntakeSubsystem intake
                    ) {
    super(
      new InstantCommand(()->System.out.println("AUTO START!")), // TODO remove these comments if they are no longer nessecary

      new ArmZero(arm),

      new InstantCommand(()->System.out.println("NEXT STEP (1)")),

      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          // move to the line and run the turret
          // new AutoLinear(drive, 84),
          new RamseteContainer(m_drivetrain, new TLine(){public double getLengthIn(){return 84;}}).getCommandAndStop(),
          new TurretPreset(turret, -195)
        ),
        // rev the shooter (and don't stop it)
        new ShooterRPM(shooter){public void end(boolean interrupted){/* This is empty is to not stop the motor from rev-ing*/}}
      ),

      new InstantCommand(()->System.out.println("NEXT STEP (2)")),

      new TurretAim(camera, turret){public void end(boolean interrupted){/* don't change the pipeline back */}},

      new ParallelCommandGroup(
        // shoot the balls
        new ParallelDeadlineGroup(
          // run the shooter and aim the turret (but the aiming will happen for as long as shooting is)
          new ShooterHopper(shooter, hopper, feed).withTimeout(2),
          new TurretAim(camera, turret){public boolean isFinished(){return false;}} // this is so that it will aim forever until the shooting is finished
        ),
        // extend the arm
        new ArmToPosition(arm, Vars.ARM_OUT)
      ),

      new InstantCommand(()->System.out.println("NEXT STEP (3)")),

      // TODO check distances
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          // drive to each ball (or pair) and stop before each one
          new RamseteContainer(m_drivetrain, new TLine(){
            public double startSpeed(){return 0;}
            public double getLengthIn(){return 38;}
            public double endSpeed(){return 50;}
          }).getCommand(),
          new RamseteContainer(m_drivetrain, new TLine(){
            public double startSpeed(){return 50;}
            public double getLengthIn(){return 50;}
            public double endSpeed(){return 0;}
          }).getCommand()
          // new RamseteContainer(m_drivetrain, new TLine(){public double getLengthIn(){return 58.4;}}).getCommandAndStop()
        ),
        // run the intake and hopper
        new IntakePower(intake, Vars.INTAKE_PERCENT),
        new HopperPower(hopper, Vars.HOPPER_WALL_PERCENT, Vars.HOPPER_FLOOR_PERCENT),
        new FeedBall(feed, Vars.FEED_PERCENT),
        new TurretAim(camera, turret){public boolean isFinished(){return false;}}, // this is so that it will aim forever until the shooting is finished
        new ShooterRPM(shooter)
      ),

      new InstantCommand(()->System.out.println("NEXT STEP (4)")),

      // pull the arm up
      new ParallelCommandGroup(
        new ArmToPosition(arm, Vars.ARM_IN), // (the player position to not squeeze balls)
        new RamseteContainer(m_drivetrain, new TLine(){public double getLengthIn(){return -72;}}).getCommandAndStop()
      ),
      
      new InstantCommand(()->System.out.println("FINISHED!"))
    );
  }
}