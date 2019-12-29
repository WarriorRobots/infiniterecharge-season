package frc.robot.commands.camera;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CameraChangePipeline extends Command {

	int pipeline;

	/**
	 * Changes the pipeline ID of the limelight, which switches the vision tracking settings.
	 * Use the static variables in CameraSubsystem.
	 */
	public CameraChangePipeline(int pipeline) {
		requires(Robot.camera);
		this.pipeline = pipeline;
	}

	@Override
	protected void initialize() {
		Robot.camera.setPipeline(pipeline);
		System.out.println("Camera: Running " + this.getClass().getSimpleName());
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
