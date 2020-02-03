/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedControllerSubsystem;

public class LEDChangePattern extends CommandBase {
  LedControllerSubsystem m_scan;
  private double channel;

  /**
   * Changes the LEDs' pattern based on given channel value
   * @param channel id as a spark value
   */
  public LEDChangePattern(double channel, LedControllerSubsystem scan) {
    m_scan = scan;
    addRequirements(this.m_scan);
    
    this.channel = channel;
  }

  @Override
  public void initialize() {
    m_scan.setChannel(channel);
    System.out.println("LED: Running " + this.getClass().getSimpleName());
  }

  @Override
	public boolean isFinished() {
		return false;
	}
}