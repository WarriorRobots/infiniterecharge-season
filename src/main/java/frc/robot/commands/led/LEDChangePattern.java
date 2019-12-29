package frc.robot.commands.led;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


public class LEDChangePattern extends Command {

  private double channel;

  /**
   * Changes the LEDs' pattern based on given channel value
   * @param channel id as a spark value
   */
  public LEDChangePattern(double channel) {
    requires(Robot.leds);
    
    this.channel = channel;
  }

  @Override
  protected void initialize() {
    Robot.leds.setChannel(channel);
    System.out.println("LED: Running " + this.getClass().getSimpleName());
  }

  @Override
	protected boolean isFinished() {
		return false;
	}
}
