package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.QuickAccessVars;
import frc.robot.Robot;

public class TurnLockDrive extends Command {

	/**
	 * A drive command for use at high speeds, which will not be sensitive to small
	 * changes in joystick position. Allows drivers to drive in straight lines
	 * without perfect control of joysticks.
	 */
	public TurnLockDrive() {
		requires(Robot.drivetrain);
	}

	@Override
	protected void initialize() {
		System.out.println("Drivetrain: Starting " + this.getClass().getSimpleName());
	}

	@Override
	protected void execute() {
		double leftSpeed = Robot.input.getLeftY();
		double rightSpeed = Robot.input.getRightY();

		double difference = Math.abs(leftSpeed - rightSpeed);
		double average = (leftSpeed + rightSpeed) / 2.0;

		// TURNLOCK_THRESHOLD is how far apart the joystick values have to be (in
		// decimal percentage) before TurnLock disables.
		if (difference < QuickAccessVars.TURNLOCK_THRESHOLD) {
			// the values are averaged and the robot drives straight
			Robot.drivetrain.tankDriveTeleop(average, average);
		} else {
			// normal TeleopTankDrive behavior
			Robot.drivetrain.tankDriveTeleop(leftSpeed, rightSpeed);
		}
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