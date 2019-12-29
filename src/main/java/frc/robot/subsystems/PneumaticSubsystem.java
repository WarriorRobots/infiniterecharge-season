package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.Constants;

/**
 * Contains all pneumatics on the robot.
 */
public class PneumaticSubsystem extends Subsystem {

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

	/**
	 * Forwards the pressure on the main solenoid.
	 */
	public void forwardMain() {
		mainSol.set(Value.kForward);
	}

	/**
	 * Reverses the pressure on the main solenoid.
	 */
	public void reverseMain() {
		mainSol.set(Value.kReverse);
	}

	/**
	 * Shuts off power to the main solenoid.
	 * Use after extending or retracting; this will not move the piston.
	 */
	public void neutralizeMain() {
		mainSol.set(Value.kOff);
	}

	/**
	 * Shuts off power to all solenoids.
	 * Use after extending or retracting; this will not move any pistons.
	 */
	public void neutralizeAll() {
		neutralizeMain();
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