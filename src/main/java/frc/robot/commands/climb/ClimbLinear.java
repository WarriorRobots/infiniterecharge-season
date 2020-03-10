/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.Brakes;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ClimbLinear extends SequentialCommandGroup {
  /**
   * A command to tell the climb to move linearly.
   * Also makes sure the brakes disengage when the command is running.
   */
  public ClimbLinear(ClimbSubsystem climb, DoubleSupplier input) {
    super(
      new ClimbBrakes(climb, Brakes.disengage),
      new SubClimbLinear(climb, input),
      new ClimbBrakes(climb, Brakes.engage)
    );
  }
}
