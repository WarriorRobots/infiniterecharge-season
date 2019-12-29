package frc.robot.commands.compressor;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

@Deprecated // do not use except for troubleshooting
public class CompressorEnable extends InstantCommand {

	/**
	 * Runs for one loop, enabling the robot compressor
	 * (which will not turn on unless there is low pressure).
	 */
	public CompressorEnable() {
		requires(Robot.pneumatics);
	}

	@Override
	protected void execute() {
		Robot.pneumatics.enableCompressor();
		System.out.println("Debug: Running " + this.getClass().getSimpleName());
	}

}