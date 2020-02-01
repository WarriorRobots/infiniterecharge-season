package frc.robot.commands.camera;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.QuickAccessVars;
import frc.robot.Robot;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.LedControllerSubsystem;
import frc.robot.util.SynchronousPIDF;

/** Approach the target keeping the target centered and stopping at a distance using 2 different PIDs. */
public class CameraStopAtDistance extends Command {

	/** PID used for approaching the target. */
	private SynchronousPIDF PIDapproach;
	/** PID for keeping the target centered */
	private SynchronousPIDF PIDcenter;

	private Timer timer;
	/** Calculated PID output from {@link #PIDapproach} should stored in value. */
	private double valueApproach;
	/** Calculated PID output from {@link #PIDcenter} should stored in value. */
	private double valueCenter;

	// states whether the command can be finished or not
	private boolean finishable;

	/**
	 * @param finishable True if the command can be stopped by being at a distance (aka true if in auto).
	 */
	public CameraStopAtDistance(boolean finishable) {
		requires(Robot.drivetrain);
		requires(Robot.camera);
		requires(Robot.leds);

		valueApproach = 0;
		valueCenter = 0;

		PIDapproach = new SynchronousPIDF(QuickAccessVars.KP_APPROACH, QuickAccessVars.KI_APPROACH,
				QuickAccessVars.KD_APPROACH);
		PIDcenter = new SynchronousPIDF(QuickAccessVars.KP_CENTER, QuickAccessVars.KI_CENTER,
				QuickAccessVars.KD_CENTER);

		this.finishable = finishable;
		timer = new Timer();
	}

	@Override
	protected void initialize() {
		System.out.println("Camera: Starting " + this.getClass().getSimpleName());
		Robot.camera.setPipeline(CameraSubsystem.PIPELINE_CENTER);

		// distance in in inches
		PIDapproach.setSetpoint(QuickAccessVars.SETPOINT_APPROACH);
		PIDcenter.setSetpoint(0); // 0 means keep the target centered

		timer.start();

	}

	@Override
	protected void execute() {

		if (Robot.camera.canSeeObject()) {
			valueApproach = PIDapproach.calculate(Robot.camera.TargetDistance("port"), timer.get());
			valueCenter = PIDcenter.calculate(Robot.camera.getObjectX()+QuickAccessVars.CAMERA_BIAS, timer.get());

			// if the robot within the specified range then RED STROBING
			if (Math.abs(Robot.camera.TargetDistance("port")-QuickAccessVars.SETPOINT_APPROACH) <
			QuickAccessVars.TOLERANCE_APPROACH) {
				Robot.leds.setChannel(LedControllerSubsystem.atTarget);
			}
			// if the robot sees the target however is not where it should be BLUE STROBING
			else {
				Robot.leds.setChannel(LedControllerSubsystem.seeTarget);
			}
		} else {
			valueCenter = 0;
			valueApproach = 0;

			// if the robot doesn't see the target it should be BLUE BREATHING
			Robot.leds.setChannel(LedControllerSubsystem.IDLE);
		}

		Robot.drivetrain.arcadeDriveRaw(-valueApproach, -valueCenter);

	}

	@Override
	protected boolean isFinished() {
		if (finishable) {
			if (Math.abs(Robot.camera.TargetDistance("port")-QuickAccessVars.SETPOINT_APPROACH) <
			QuickAccessVars.TOLERANCE_APPROACH) {
				// returns true if the command is finishable and the camera is at the right distance away
				return true;
			}
		}
		// returns false if the commmand is not finishable or if it is not at the tolerable zone to say it's finished
		return false;
    }

	@Override
	protected void end() {
        System.out.println("Camera: Finishing " + this.getClass().getSimpleName());
		timer.stop();
		PIDapproach.reset();
		PIDcenter.reset();
		valueApproach = 0;
		valueCenter = 0;
		Robot.drivetrain.stopDrive();
    }

    @Override
	protected void interrupted() {
        System.out.println("Camera: Canceling " + this.getClass().getSimpleName());
		timer.stop();
		PIDapproach.reset();
		PIDcenter.reset();
		valueApproach = 0;
		valueCenter = 0;
		Robot.drivetrain.stopDrive();
    }
    
    

}