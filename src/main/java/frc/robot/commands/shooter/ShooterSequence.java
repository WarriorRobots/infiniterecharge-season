/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShooterSequence extends SequentialCommandGroup {
  /**
   * A command that clears the shooter, revs the shooter, and when the shooter is up to speed, feeds the shooter.
   */
  public ShooterSequence(ShooterSubsystem shooter, HopperSubsystem hopper, FeedSubsystem feed) {
    super(
      new ShooterPrep(shooter, hopper, feed),
      new ShooterHopper(shooter, hopper, feed)
    );
  }
}
