package frc.robot.util.triggers;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Button;


/**
 * Utility class that turns analog joystick/trigger inputs into boolean outputs.
 * <p>
 * Allows use of whenPressed(), whileHeld(), etc, which normally require
 * true/false inputs.
 */
public class ThresholdTrigger extends Button {

	private DoubleSupplier input;
	private BooleanSupplier disableEvent;
	private double threshold;
	private boolean ignoreDirection;

	/**
	 * Create a trigger with the following parameters:
	 * 
	 * @param input           A lambda function that returns a boolean value.
	 * @param threshold       If the input exceeds this value, this trigger returns
	 *                        true.
	 * @param ignoreDirection If this is true, input is made positive.
	 * @param disableEvent    If this lambda function returns true, the trigger
	 *                        returns false.
	 */
	public ThresholdTrigger(DoubleSupplier input, double threshold, boolean ignoreDirection,
			BooleanSupplier disableEvent) {
		this.input = input;
		this.threshold = threshold;
		this.ignoreDirection = ignoreDirection;
		this.disableEvent = disableEvent;
	}

	/**
	 * Create a trigger with the following parameters:
	 * 
	 * @param input     A lambda function that returns a boolean value.
	 * @param threshold If the input exceeds this value, this trigger returns true.
	 */
	public ThresholdTrigger(DoubleSupplier input, double threshold) {
		this(input, threshold, true, () -> false);
	}

	/**
	 * Create a trigger with the following parameters:
	 * 
	 * @param input           A lambda function that returns a boolean value.
	 * @param threshold       If the input exceeds this value, this trigger returns
	 *                        true.
	 * @param ignoreDirection If this is true, input is made positive.
	 */
	public ThresholdTrigger(DoubleSupplier input, double threshold, boolean ignoreDirection) {
		this(input, threshold, ignoreDirection, () -> false);

	}

	/**
	 * Create a trigger with the following parameters:
	 * 
	 * @param input        A lambda function that returns a boolean value.
	 * @param threshold    If the input exceeds this value, this trigger returns
	 *                     true.
	 * @param disableEvent If this lambda function returns true, the trigger returns
	 *                     false.
	 */
	public ThresholdTrigger(DoubleSupplier input, double threshold, BooleanSupplier disableEvent) {
		this(input, threshold, true, disableEvent);
	}

	@Override
	public boolean get() {
		double val = input.getAsDouble();
		if (disableEvent.getAsBoolean()) {
			return false;
		} else {
			if (ignoreDirection) {
				return Math.abs(val) > Math.abs(threshold);
			} else {
				if (threshold < 0) {
					return val < threshold;
				} else {
					return val > threshold;
				}
			}
		}
	}

}