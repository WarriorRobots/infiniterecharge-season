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
import frc.robot.Vars;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.arm.ArmZero;
import frc.robot.commands.auto.trajectories.TAutoHarvestAltPoint;
import frc.robot.commands.auto.trajectories.TAutoHarvestAltShoot;
import frc.robot.commands.auto.trajectories.TLine;
import frc.robot.commands.intake.IntakeHopper;
import frc.robot.commands.intake.IntakePower;
import frc.robot.commands.shooter.ShooterHopper;
import frc.robot.commands.shooter.ShooterPrep;
import frc.robot.commands.shooter.ShooterRPM;
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
public class AutoHarvestAlt extends SequentialCommandGroup {
  /**
   * 6-8 ball auto.
   * <p>
   * Move to the trench and pick up a ball.
   * Shoot 4 balls at the start of the trench.
   * Pick up 2 balls from the trench/color wheel.
   * Pick up 2 balls from the center.
   * Shoot 4 balls.
   */
  public AutoHarvestAlt(
                      DrivetrainSubsystem drive,
                      ShooterSubsystem shooter,
                      TurretSubsystem turret,
                      CameraSubsystem camera,
                      FeedSubsystem feed,
                      HopperSubsystem hopper,
                      ArmSubsystem arm,
                      IntakeSubsystem intake
                    ) {
    super(
      new InstantCommand(()->System.out.println("AUTO START!")),

      new ArmZero(arm),

      new InstantCommand(()->System.out.println("NEXT STEP (1)")),

      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          // move to the line and run the turret
          new RamseteContainer(drive, new TLine(){public double getLengthIn(){return Vars.HARVEST_TO_TRENCH;}}).getCommandAndStop(),
          new TurretPreset(turret, Vars.HARVEST_TURRET),
          // extend the arm
          new ArmToPosition(arm, Vars.ARM_OUT)
        ),
        // rev the shooter (and don't stop it)
        new ShooterRPM(shooter){public void end(boolean interrupted){/* This is empty is to not stop the motor from rev-ing*/}},
        // run the intake and hopper
        new IntakePower(intake, Vars.INTAKE_PERCENT)
      ),

      new InstantCommand(()->System.out.println("NEXT STEP (2)")),

      new TurretAim(camera, turret){public void end(boolean interrupted){/* don't change the pipeline back */}},

      new ParallelDeadlineGroup(
        // run the shooter and aim the turret (but the aiming will happen for as long as shooting is)
        new ShooterHopper(shooter, intake, hopper, feed).withTimeout(Vars.HARVEST_SHOOT_TIME_START),
        new TurretAim(camera, turret){public boolean isFinished(){return false;}} // this is so that it will aim forever until the shooting is finished
      ),

      new InstantCommand(()->System.out.println("NEXT STEP (3)")),

      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          // drive to each ball (or pair) and stop before each one
          new RamseteContainer(drive, new TLine(){
            public double startSpeed(){return 0;}
            public double getLengthIn(){return Vars.HARVEST_LINE_1;}
            public double endSpeed(){return Vars.HARVEST_SLOW;}
          }).getCommand(),
          new RamseteContainer(drive, new TLine(){
            public double startSpeed(){return Vars.HARVEST_SLOW;}
            public double getLengthIn(){return Vars.HARVEST_LINE_2;}
            public double endSpeed(){return Vars.HARVEST_SLOW;}
          }).getCommand()
        ),
        // run the intake and hopper
        new IntakeHopper(intake, hopper, feed)
      ),

      new InstantCommand(()->System.out.println("NEXT STEP (4)")),

      new RamseteContainer(drive, new TLine(){public double getLengthIn() {return Vars.HARVESTALT_RETURN;}}).getCommand(),

      new ParallelDeadlineGroup(
        new RamseteContainer(drive, new TAutoHarvestAltPoint()).getCommand(),
        new IntakeHopper(intake, hopper, feed)
      ),

      new ParallelDeadlineGroup(
        // drive to the shoot location
        new RamseteContainer(drive, new TAutoHarvestAltShoot()).getCommand(),
        // aim the turret
        new TurretAim(camera, turret){public boolean isFinished(){return false;}}, // this is so that it will aim forever until the shooting is finished
        // prep and rev the shooter
        new SequentialCommandGroup(
          new ShooterPrep(hopper, feed),
          new ShooterRPM(shooter){public void end(boolean interrupted){/* This is empty is to not stop the motor from rev-ing*/}}
        )
      ),

      new ParallelDeadlineGroup(
        // run the shooter and aim the turret (but the aiming will happen for as long as shooting is)
        new ShooterHopper(shooter, intake, hopper, feed).withTimeout(Vars.HARVEST_SHOOT_TIME_START),
        new TurretAim(camera, turret){public boolean isFinished(){return false;}} // this is so that it will aim forever until the shooting is finished
      ),
      
      new InstantCommand(()->System.out.println("FINISHED!"))
    );
  }
}
