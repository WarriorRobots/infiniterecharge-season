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
import frc.robot.commands.auto.trajectories.TLine;
import frc.robot.commands.drive.AutoAngular;
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
   * Robot faces forwards towards the trench (84") and turns to face the target (~ -220),
   * shoots the balls for ~ 3 seconds,
   * drives forwards (115") and (put the arm down then runs the intake the whole time),
   * go backwards to the back of the trench again (115")
   * shoots the balls for ~ 5
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
      new InstantCommand(()->System.out.println("AUTO START!")), // TODO remove these comments if they are no longer nessecary

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
        new ShooterHopper(shooter, hopper, feed).withTimeout(Vars.HARVEST_SHOOT_TIME_START),
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
          }).getCommand(),
          new RamseteContainer(drive, new TLine(){
            public double startSpeed(){return Vars.HARVEST_SLOW;}
            public double getLengthIn(){return Vars.HARVEST_LINE_3;}
            public double endSpeed(){return 0;}
          }).getCommand()
          // new RamseteContainer(m_drivetrain, new TLine(){public double getLengthIn(){return 58.4;}}).getCommandAndStop()
        ),
        // run the intake and hopper
        new IntakeHopper(intake, hopper, feed)
      ),

      new InstantCommand(()->System.out.println("NEXT STEP (4)")),

      new ParallelDeadlineGroup(
        // pull the arm up
        new ParallelCommandGroup(
          new ArmToPosition(arm, Vars.ARM_IN), // (the player position to not squeeze balls)
          new RamseteContainer(drive, new TLine(){public double getLengthIn(){return Vars.HARVEST_RETURN;}}).getCommandAndStop()
        ),
        // new ShooterRPM(shooter){public void end(boolean interrupted){/* This is empty is to not stop the motor from rev-ing*/}},
        new TurretAim(camera, turret){public boolean isFinished(){return false;}}, // this is so that it will aim forever until the shooting is finished
        new SequentialCommandGroup(
          new ShooterPrep(shooter, hopper, feed),
          new ShooterRPM(shooter){public void end(boolean interrupted){/* This is empty is to not stop the motor from rev-ing*/}}
        )
      ),

      new InstantCommand(()->System.out.println("NEXT STEP (5)")),

      // shoot the balls
      new ParallelDeadlineGroup(
        // run the shooter and aim the turret (but the aiming will happen for as long as shooting is)
        new ShooterHopper(shooter, hopper, feed).withTimeout(Vars.HARVEST_SHOOT_TIME_END),
        new TurretAim(camera, turret){public boolean isFinished(){return false;}} // this is so that it will aim forever until the shooting is finished
      ),
      
      new InstantCommand(()->System.out.println("FINISHED!"))
    );
  }
}
