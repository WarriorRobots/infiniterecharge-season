package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.shooter.ShooterRPM;
// command imports
import frc.robot.util.triggers.DpadTrigger;
import frc.robot.util.triggers.ThresholdTrigger;

/**
 * Contains methods for receiving data from Joysticks and the Xbox controller.
 * Also contains all keybinds and commands.
 */

 // JOSE CODE JOSE CODE
 // On left joystick 0, while held down, fire 

@SuppressWarnings("unused")
public final class ControlHandler {

	private static final int LEFT_JOY = 1;
	private static final int RIGHT_JOY = 0;
	private static final int XBOX = 2;

	// JOSE CODE JOSE CODE using trigger button
	private Joystick leftJoy;
	private JoystickButton leftJoyTriggerButton, leftJoyThumbButton, leftJoyButton3, leftJoyButton4, leftJoyButton5,
			leftJoyButton6, leftJoyButton7, leftJoyButton8, leftJoyButton9, leftJoyButton10, leftJoyButton11,
			leftJoyButton12;
	private Joystick rightJoy;
	private JoystickButton rightJoyTriggerButton, rightJoyThumbButton, rightJoyButton3, rightJoyButton4,
			rightJoyButton5, rightJoyButton6, rightJoyButton7, rightJoyButton8, rightJoyButton9, rightJoyButton10,
			rightJoyButton11, rightJoyButton12;

	private XboxController xbox;
	private ThresholdTrigger leftXboxTrigger, rightXboxTrigger, leftXboxJoyUp, leftXboxJoyDown, rightXboxJoyUp,
			rightXboxJoyDown;
	private JoystickButton xboxX, xboxY, xboxB, xboxA, xboxSTART, xboxSELECT, leftXboxBumper, rightXboxBumper, xboxL3,
			xboxR3;
	private DpadTrigger xboxUp, xboxDown, xboxLeft, xboxRight;

	public ControlHandler() {
		leftJoy = new Joystick(LEFT_JOY);
		leftJoyTriggerButton = new JoystickButton(leftJoy, 1);
		leftJoyThumbButton = new JoystickButton(leftJoy, 2);
		leftJoyButton3 = new JoystickButton(leftJoy, 3);
		leftJoyButton4 = new JoystickButton(leftJoy, 4);
		leftJoyButton5 = new JoystickButton(leftJoy, 5);
		leftJoyButton6 = new JoystickButton(leftJoy, 6);
		leftJoyButton7 = new JoystickButton(leftJoy, 7);
		leftJoyButton8 = new JoystickButton(leftJoy, 8);
		leftJoyButton9 = new JoystickButton(leftJoy, 9);
		leftJoyButton10 = new JoystickButton(leftJoy, 10);
		leftJoyButton11 = new JoystickButton(leftJoy, 11);
		leftJoyButton12 = new JoystickButton(leftJoy, 12);

		rightJoy = new Joystick(RIGHT_JOY);
		rightJoyTriggerButton = new JoystickButton(rightJoy, 1);
		rightJoyThumbButton = new JoystickButton(rightJoy, 2);
		rightJoyButton3 = new JoystickButton(rightJoy, 3);
		rightJoyButton4 = new JoystickButton(rightJoy, 4);
		rightJoyButton5 = new JoystickButton(rightJoy, 5);
		rightJoyButton6 = new JoystickButton(rightJoy, 6);
		rightJoyButton7 = new JoystickButton(rightJoy, 7);
		rightJoyButton8 = new JoystickButton(rightJoy, 8);
		rightJoyButton9 = new JoystickButton(rightJoy, 9);
		rightJoyButton10 = new JoystickButton(rightJoy, 10);
		rightJoyButton11 = new JoystickButton(rightJoy, 11);
		rightJoyButton12 = new JoystickButton(rightJoy, 12);

		xbox = new XboxController(XBOX);
		xboxA = new JoystickButton(xbox, 1);
		xboxB = new JoystickButton(xbox, 2);
		xboxX = new JoystickButton(xbox, 3);
		xboxY = new JoystickButton(xbox, 4);
		leftXboxBumper = new JoystickButton(xbox, 5);
		rightXboxBumper = new JoystickButton(xbox, 6);
		xboxSTART = new JoystickButton(xbox, 8);
		xboxSELECT = new JoystickButton(xbox, 7);
		xboxL3 = new JoystickButton(xbox, 9);
		xboxR3 = new JoystickButton(xbox, 10);
		leftXboxTrigger = new ThresholdTrigger(() -> xbox.getTriggerAxis(Hand.kLeft), 0.5);
		rightXboxTrigger = new ThresholdTrigger(() -> xbox.getTriggerAxis(Hand.kRight), 0.5);
		xboxUp = new DpadTrigger(() -> xbox.getPOV(), 0);
		xboxDown = new DpadTrigger(() -> xbox.getPOV(), 180);
		xboxRight = new DpadTrigger(() -> xbox.getPOV(), 90);
		xboxLeft = new DpadTrigger(() -> xbox.getPOV(), 270);
		leftXboxJoyUp = new ThresholdTrigger(
			() -> -xbox.getY(Hand.kLeft),
			QuickAccessVars.XBOX_JOYSTICK_THRESHOLD,
			false,
			() -> xbox.getStickButton(Hand.kLeft)
		);
		leftXboxJoyDown = new ThresholdTrigger(
			() -> -xbox.getY(Hand.kLeft),
			-QuickAccessVars.XBOX_JOYSTICK_THRESHOLD,
			false,
			() -> xbox.getStickButton(Hand.kLeft)
		);
		rightXboxJoyUp = new ThresholdTrigger(
			() -> -xbox.getY(Hand.kRight),
			QuickAccessVars.XBOX_JOYSTICK_THRESHOLD,
			false,
			() -> xbox.getStickButton(Hand.kRight)
		);
		rightXboxJoyDown = new ThresholdTrigger(
			() -> -xbox.getY(Hand.kRight),
			-QuickAccessVars.XBOX_JOYSTICK_THRESHOLD,
			false,
			() -> xbox.getStickButton(Hand.kRight)
		);
		// JOSE CODE JOSE CODE it shoots the ball
		leftJoyTriggerButton.whileHeld(new ShooterRPM()); 




		
		// TODO Figure out the binding and commands that are to be used
		// eg. button.whenPressed(new Command());
	}

	// -----------------------------------------------------------------//

	/**
	 * Gets Y-value of left joystick multiplied by scalingFactor.
	 * 
	 * @param scalingFactor Decimal value that proportionally scales output.
	 */
	public double getLeftY(double scalingFactor) {
		// -1 because the joystick reads -1 when pushed forwards so this makes forwards 1
		return leftJoy.getY() * scalingFactor * -1;
	}

	/**
	 * Gets Y-value of left joystick.
	 */
	public double getLeftY() {
		return this.getLeftY(1);
	}

	/**
	 * Gets Y-value of right joystick multiplied by scalingFactor.
	 * 
	 * @param scalingFactor Decimal value that proportionally scales output.
	 */
	public double getRightY(double scalingFactor) {
		// -1 because the joystick reads -1 when pushed forwards so this makes forwards 1
		return rightJoy.getY() * scalingFactor * -1;
	}

	/**
	 * Gets Y-value of right joystick.
	 */
	public double getRightY() {
		return this.getRightY(1);
	}

	// -----------------------------------------------------------------//

	/**
	 * Gets X-value of left joystick multiplied by scalingFactor.
	 * 
	 * @param scalingFactor Decimal value that proportionally scales output.
	 */
	public double getLeftX(double scalingFactor) {
		return leftJoy.getX() * scalingFactor;
	}

	/**
	 * Gets X-value of left joystick.
	 */
	public double getLeftX() {
		return this.getLeftX(1);
	}

	/**
	 * Gets X-value of right joystick multiplied by scalingFactor.
	 * 
	 * @param scalingFactor Decimal value that proportionally scales output.
	 */
	public double getRightX(double scalingFactor) {
		return rightJoy.getX() * scalingFactor;
	}

	/**
	 * Gets X-value of right joystick.
	 */
	public double getRightX() {
		return this.getRightX(1);
	}

	// -----------------------------------------------------------------//

	/**
	 * Gets Y-value of left Xbox joystick multiplied by scalingFactor.
	 * 
	 * @param scalingFactor Decimal value that proportionally scales output.
	 */
	public double getXboxLeftY(double scalingFactor) {
		return xbox.getY(Hand.kLeft) * scalingFactor * -1.0; // inverted so that up is positive
	}

	/**
	 * Gets Y-value of left Xbox joystick.
	 */
	public double getXboxLeftY() {
		return this.getXboxLeftY(1);
	}

	/**
	 * Gets X-value of left Xbox joystick multiplied by scalingFactor.
	 * 
	 * @param scalingFactor Decimal value that proportionally scales output.
	 */
	public double getXboxLeftX(double scalingFactor) {
		return xbox.getX(Hand.kLeft) * scalingFactor;
	}

	/**
	 * Gets X-value of left Xbox joystick.
	 */
	public double getXboxLeftX() {
		return this.getXboxLeftX(1);
	}

	// -----------------------------------------------------------------//

	/**
	 * Gets Y-value of right Xbox joystick multiplied by scalingFactor.
	 * 
	 * @param scalingFactor Decimal value that proportionally scales output.
	 */
	public double getXboxRightY(double scalingFactor) {
		return xbox.getY(Hand.kRight) * scalingFactor * -1.0; // inverted so that up is positive
	}

	/**
	 * Gets Y-value of right Xbox joystick.
	 */
	public double getXboxRightY() {
		return this.getXboxRightY(1);
	}

	/**
	 * Gets X-value of right Xbox joystick multiplied by scalingFactor.
	 * 
	 * @param scalingFactor Decimal value that proportionally scales output.
	 */
	public double getXboxRightX(double scalingFactor) {
		return xbox.getX(Hand.kRight) * scalingFactor;
	}

	/**
	 * Gets X-value of right Xbox joystick.
	 */
	public double getXboxRightX() {
		return this.getXboxRightX(1);
	}

	// -----------------------------------------------------------------//

	/**
	 * Get the angle, in degrees, that the D-pad buttons are currently pressed in.
	 * <p>
	 * 0 degrees is up, and angle increases in a clockwise direction.
	 * <p>
	 * <b>WARNING:</b> If the Xbox is unplugged, the code thinks UP is being
	 * pressed!
	 */
	public double getDpadAngle() {
		return xbox.getPOV();
	}

}