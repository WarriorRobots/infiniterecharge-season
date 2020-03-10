/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.Brakes;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ClimbToPosition extends SequentialCommandGroup {
  /**
   * A command to make the climb move to a set position. Ends after the climb reaches said position.
   * Also makes sure the brakes disengage when the command is running.
   */
  public ClimbToPosition(ClimbSubsystem climb, double position) {
    super(
      new ClimbBrakes(climb, Brakes.disengage),
      new SubClimbToPosition(climb, position),
      new ClimbBrakes(climb, Brakes.engage)
    );
  }
}
