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

public class HopperGroupPower extends ParallelCommandGroup {
  /**
   * Run the hopper and feed at some percent
   * @param hopper Hopper subsystem
   * @param feed Feed subsystem
   * @param hopperwall_percent Percent output of wall when running this command.
   * @param hopperfloor_percent Percent output of floor when running this command.
   * @param feed_percent Percent output of feed when running this command.
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
