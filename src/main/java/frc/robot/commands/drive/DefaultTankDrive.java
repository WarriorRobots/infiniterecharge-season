package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DefaultTankDrive extends Command {

	/**
	 * Push the left joystick vertically to drive the left wheels.
	 * Push the right joystick vertically to drive the right wheels.
	 */
	public DefaultTankDrive() {
		requires(Robot.drivetrain);
	}

	@Override
	protected void initialize() {
		System.out.println("Drivetrain: Starting " + this.getClass().getSimpleName());
	}

	@Override
	protected void execute() {
		Robot.drivetrain.tankDriveTeleop(Robot.input.getLeftY(), Robot.input.getRightY());
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {
		Robot.drivetrain.stopDrive();
		System.out.println("Drivetrain: Finishing " + this.getClass().getSimpleName());
	}

	@Override
	protected void interrupted() {
		Robot.drivetrain.stopDrive();
		System.out.println("Drivetrain: Canceling " + this.getClass().getSimpleName());
	}
}
