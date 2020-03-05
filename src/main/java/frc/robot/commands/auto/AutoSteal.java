/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmZero;
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
public class AutoSteal extends SequentialCommandGroup {
  /**
   * A command to take the two balls from the opposing trech, fire them, and go to the center to get more balls.
   */
  public AutoSteal(
                  DrivetrainSubsystem m_drivetrain,
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

      // go into opposing trench
      // & put down arm
      // & run intake

      // go back (a little)
      // & pull arm up

      // go to some arbitrary spot in the middle of the field (to shoot)
      // & prepare the turret
      // then prepare the camera

      // arm down
      // shoot for x seconds
      // & keep the camera on target

      // move to the middle island
      // then "sweep" the middle for balls
      // then move to the other side
      // then "sweep"
      // & run the intake

      // move to shoot position
      // & arm up
      // & prepare turret
      // then prepare camera

      // shoot for x seconds
      // & keep the camera on target

      new InstantCommand(()->System.out.println("FINISHED!"))
    );
  }
}
