package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.QuickAccessVars;
import frc.robot.Robot;

public class ArcadeDrive extends Command {

	/**
	 * Push the right joystick vertically to drive forwards/backwards,
	 * and horizontally to turn.
	 */
	public ArcadeDrive() {
		requires(Robot.drivetrain);
	}

	@Override
	protected void initialize() {
		System.out.println("Drivetrain: Starting " + this.getClass().getSimpleName());
	}

	@Override
	protected void execute() {
		Robot.drivetrain.arcadeDriveTeleop(Robot.input.getRightY(QuickAccessVars.ARCADE_FORWARD_MODIFIER),
				Robot.input.getRightX(QuickAccessVars.ARCADE_TURN_MODIFIER));
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
