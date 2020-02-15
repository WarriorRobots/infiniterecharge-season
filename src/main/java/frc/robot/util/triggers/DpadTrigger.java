package frc.robot.util.triggers;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.button.Button;


/**
 * Utility class that converts D-pad inputs into true/false outputs.
 * <p>
 * Allows use of whenPressed(), whileHeld(), etc, which normally require
 * true/false inputs.
 */
public class DpadTrigger extends Button {

	private Supplier<Integer> input;
	private int angle;

	/**
	 * Create a trigger with the following parameters:
	 * 
	 * @param input A lambda function that returns the angular direction of a D-pad
	 *              button press. Try <code>() -> xbox.getPOV()</code>
	 * @param angle The angle required for the trigger to return true.
	 *              <code>-1</code> means the D-pad is not being pressed.
	 */
	public DpadTrigger(Supplier<Integer> input, int angle) {
		this.angle = angle;
		this.input = input;
	}

	@Override
	public boolean get() {
		return (input.get() == angle) ? true : false;
	}

}