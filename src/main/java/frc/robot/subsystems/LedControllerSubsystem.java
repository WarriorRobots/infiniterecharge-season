/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LedController sends PWM signals to the REV Blinkin to change the color of LEDs
 */
public class LedControllerSubsystem extends SubsystemBase {

  private static final int LED_CONTROLLER_ID = 0; // move to RobotMap.java

  /** LED controller acts as a Spark motor controller for it's inputs for changing patterns. */
  private Spark LED_controller;
  
  // all available patterns are at http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf#page=14

  /**
   * Spark value for color of the LED strip when there is no other command.
   * <p>
   * 43 Fixed Pallete Pattern Breath, Blue
   */
  public static final double IDLE = -0.15;

  // check with drivers what indicators are needed
  


  public LedControllerSubsystem() {
    LED_controller = new Spark(LED_CONTROLLER_ID);
  }

  /**
   * Change the pattern of the LED Controller by using preset channel number in {@link LedControllerSubsystem}.
   * @param channel id as a spark value
   */
  public void setChannel(double channel) {
    LED_controller.set(channel);
  }

  /**
   * Gets the spark value that the LED Controller is currently using
   * @return spark value of current pattern
   */
  public double getChannel() {
    return LED_controller.get();
  }

  @Override
	public void periodic() {}
}