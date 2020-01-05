package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.Constants;
import frc.robot.QuickAccessVars;

/**
 * Contains all pneumatics on the robot.
 */
public class PneumaticSubsystem extends IterSubsystem {

	// TODO Find solenoid forward and reverse channels
	private static final int MAIN_FORWARD = 0;
	private static final int MAIN_REVERSE = 7;

	private Compressor compressor;
	private DoubleSolenoid mainSol;

	/**
	 * Instantiates new subsystem; make ONLY ONE.
	 * <p>
	 * <code> public static final HatchPlacerSubsystem hatchPlacer = new
	 * HatchPlacerSubsystem();
	 */
	public PneumaticSubsystem() {
		compressor = new Compressor(Constants.PCM_1);
		compressor.start();

		mainSol = new DoubleSolenoid(Constants.PCM_1, MAIN_FORWARD, MAIN_REVERSE);
	}


	static class PERIODICio {

		/** Value to be commanded to the main solenoid (where the counter > 0). */
		static Value mainValue = Value.kOff;
		/** Number of loops left to command the main Solenoid. */
		static int mainCounter = 0;

	}

	public void onLoop(double t) {
		// if a solenoids counter is positive, then it needs to send the command to the solenoid
		if (PERIODICio.mainCounter > 0) {
			// a solenoid
			mainSol.set(PERIODICio.mainValue);
			PERIODICio.mainCounter--;
		}
	}

	public void onStart(double t) {/* the compressor turns on without being called */}
	public void onEnd(double t) {/* none */}
	public void periodic(double t) {/* none */}
	public void disabled(double t) {/* none */}

	
	/**
	 * Forwards the pressure on the main solenoid.
	 */
	public void forwardMain() {
		// sets the value to be written to the solenoid and sets the counter to be positive
		// (all other solenoid functions below act similarly)
		PERIODICio.mainValue = Value.kForward;
		PERIODICio.mainCounter = QuickAccessVars.PNEUMATIC_LOOP_COUNT;
	}

	/**
	 * Reverses the pressure on the main solenoid.
	 */
	public void reverseMain() {
		PERIODICio.mainValue = Value.kReverse;
		PERIODICio.mainCounter = QuickAccessVars.PNEUMATIC_LOOP_COUNT;
	}

	/**
	 * Shuts off power to the main solenoid.
	 * Use after extending or retracting; this will not move the piston.
	 */
	public void neutralizeMain() {
		PERIODICio.mainValue = Value.kOff;
		PERIODICio.mainCounter = QuickAccessVars.PNEUMATIC_LOOP_COUNT;
	}

	/**
	 * @return The current value of the main solenoid.
	 */
	public Value getMain() {
		return mainSol.get();
	}

	/**
	 * Shuts off power to all solenoids.
	 * Use after extending or retracting; this will not move any pistons.
	 */
	public void neutralizeAll() {
		neutralizeMain();
		//... (any more solenoids go here)
	}


	/**
	 * Allows the compressor to pump air at low pressures (not all the time).
	 */
	public void enableCompressor() {
		compressor.start();
	}

	/**
	 * Prevents the compressor from pumping air, at any time.
	 */
	public void disableCompressor() {
		compressor.stop();
	}


	@Override
	protected void initDefaultCommand() {
		// none
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		// builder.addBooleanProperty("low pressure?", () -> !compressor.getPressureSwitchValue(), null);
		// builder.addBooleanProperty("compressor?", () -> compressor.enabled(), null);
		// builder.addStringProperty("scissors", () -> scissorSol.get().toString(), null);
		// builder.addStringProperty("hatchpickup", () -> pickupSol.get().toString(), null);
	}

}