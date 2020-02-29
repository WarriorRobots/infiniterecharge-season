/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoLinear;
import frc.robot.commands.turret.TurretPreset;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoHarvestTest extends SequentialCommandGroup {
  /**
   * Robot faces forwards towards the trench (84") and turns to face the target (~ -220),
   * shoots the balls for ~ 3 seconds,
   * drives forwards (115") and (put the arm down then runs the intake the whole time),
   * go backwards to the back of the trench again (115")
   * shoots the balls for ~ 5
   */
  public AutoHarvestTest(DrivetrainSubsystem drive, TurretSubsystem turret) {
    super(
      new ParallelCommandGroup(
        new AutoLinear(drive, 84),
        new TurretPreset(turret, -220)
      )
    );
  }
}
