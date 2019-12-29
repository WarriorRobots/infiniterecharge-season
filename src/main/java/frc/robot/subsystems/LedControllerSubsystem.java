package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.commands.ChangeLEDPattern;

/**
 * LedController sends PWM signals to the REV Blinkin to change the color of LEDs
 */
public class LedControllerSubsystem extends Subsystem {

  // TODO Check what PWM the LED Controller is plugged into
  private static final int LED_CONTROLLER_ID = 0;

  /** LED controller acts as a Spark motor controller for it's inputs for changing patterns. */
  private Spark LED_controller;

  
  // all available patterns are at http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf#page=14

  /**
   * Spark value for color of the LED strip when there is no other command.
   * <p>
   * 43 Fixed Pallete Pattern Breath, Blue
   */
  public static final double IDLE = -0.15;

  // TODO check with drivers what indicators are needed
  


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
  public void initDefaultCommand() {
    setDefaultCommand(new ChangeLEDPattern(IDLE));
  }

  @Override
	public void initSendable(SendableBuilder builder) {
    // builder.addDoubleProperty("channel", () -> getChannel(), null);
  }
}
