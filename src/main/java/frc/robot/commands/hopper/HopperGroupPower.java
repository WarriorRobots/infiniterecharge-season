/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.feed.FeedBall;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.HopperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class HopperGroupPower extends ParallelCommandGroup {
  /**
   * Creates a new HopperGroupPower.
   */
  public HopperGroupPower(HopperSubsystem hopper,
                          FeedSubsystem feed,
                          double hopperwall_percent,
                          double hopperfloor_percent,
                          double feed_percent) {
    super(
      new HopperPower(hopper, hopperwall_percent, hopperfloor_percent),
      new FeedBall(feed, feed_percent)
    );
  }
}
