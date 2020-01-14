package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.Constants;
import frc.robot.QuickAccessVars;
import frc.robot.commands.drive.DefaultTankDrive;

/**
 * Contains the drivetrain, the encoders for the left and right wheels, and the NavX gyroscope.
 */
public class DrivetrainSubsystem extends IterSubsystem {

	// TODO Make sure Falcon 500s have these ids
	private static final int LEFT_FRONT_ID = 1;
	private static final int LEFT_BACK_ID = 2;
	private static final int RIGHT_FRONT_ID = 3;
	private static final int RIGHT_BACK_ID = 4;
	
	private WPI_TalonFX leftFront, leftBack, rightFront, rightBack;
	private SpeedControllerGroup leftGroup, rightGroup;
	private DifferentialDrive differentialDrive;
	
	private AHRS navx;
	
	private enum DriveControlStates {
		OPEN_LOOP, // following controls from commands
		PATH_FOLLOWING, // following controls from path
	}
	
	// private Path currentPath = null;
	// private PathFollower pathFollower;
	private DriveControlStates currentDriveControlState;
	
	/** A Grayhill encoder has {@value} clicks per revolution. */
	// public static final int CLICKS_PER_REV = 128; TODO change this to be the talon units

	/** The robot wheel is {@value} inches in diameter. */
	public static final double WHEEL_DIAMETER = 6.0;

	/** The maximum capable velocity is {@value} inches/sec */
	public static final double MAX_VELOCITY = 114; // TODO tune for 2020 robot

	/** The robot wheel is {@value} inches in radius. */
	public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;

	/** The distance between the left and right wheels is {@value} inches. */
	public static final double TRACK_WIDTH = 26.0;
	
	/** The distance between the left and right wheels is {@value} meters. */
	public static final double TRACK_WIDTH_METER = TRACK_WIDTH / 2.0 * 0.0254;
	
	/** The robot's track's scrub factor. (unitless) */
    public static final double SCRUB_FACTOR = 1.0469745223; // TODO Change this from 254 to 2478 scrub factor

	/**
	 * The robot travels {@value} inches per encoder click.
	 */
	// Diameter * PI = circumference
	// circumference divided by clicks = distance per click.
	public static final double INCHES_DRIVEN_PER_CLICK = (WHEEL_DIAMETER * Math.PI) / CLICKS_PER_REV;
	
	/**
	 * Instantiates new subsystem; make ONLY ONE.
	 * <p>
	 * <code> public static final DrivetrainSubsystem drivetrain = new
	 * DrivetrainSubsystem();
	 */
	public DrivetrainSubsystem() {
		leftFront = new WPI_TalonFX(LEFT_FRONT_ID);
		leftBack = new WPI_TalonFX(LEFT_BACK_ID);
		leftFront.configOpenloopRamp(QuickAccessVars.DRIVETRAIN_RAMPRATE, Constants.TIMEOUT_MS);
		leftBack.configOpenloopRamp(QuickAccessVars.DRIVETRAIN_RAMPRATE, Constants.TIMEOUT_MS);
		
		rightFront = new WPI_TalonFX(RIGHT_FRONT_ID);
		rightBack = new WPI_TalonFX(RIGHT_BACK_ID);
		rightFront.configOpenloopRamp(QuickAccessVars.DRIVETRAIN_RAMPRATE, Constants.TIMEOUT_MS);
		rightBack.configOpenloopRamp(QuickAccessVars.DRIVETRAIN_RAMPRATE, Constants.TIMEOUT_MS);
		
		leftGroup = new SpeedControllerGroup(leftFront, leftBack);
		rightGroup = new SpeedControllerGroup(rightFront, rightBack);
		leftGroup.setInverted(QuickAccessVars.LEFT_SIDE_REVERSED);
		rightGroup.setInverted(QuickAccessVars.RIGHT_SIDE_REVERSED);

		differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
		differentialDrive.setSafetyEnabled(false);

		// if NavX is missing, this code will handle errors and prevent a crash
		try {
			navx = new AHRS(I2C.Port.kMXP);
		} catch (Exception ex) {
			DriverStation.reportError(ex.getMessage(), true);
		
		leftFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PID_ID, Constants.TIMEOUT_MS);
		rightFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PID_ID, Constants.TIMEOUT_MS);
		
		leftFront.setSensorPhase(QuickAccessVars.LEFT_SIDE_ENCODER_REVERSED);
		rightFront.setSensorPhase(QuickAccessVars.LEFT_SIDE_ENCODER_REVERSED);
		
		// Drive state starts out as Open loop, following driver commands or voltage commands
		currentDriveControlState = DriveControlStates.OPEN_LOOP;
	}
	
	
	static class PERIODICio {

		static int left_encoder=0;
		static int right_encoder=0;
		static double angle=0;

		static int left_velocity_ticks_per_loop=0; // clicks/second
		static int right_velocity_ticks_per_loop=0; // clicks/second

	}
	

	public void onLoop(double t) {
		// TODO put the pathfinding code here
	}
	
	public void periodic(double t) {

		PERIODICio.left_encoder = leftEnc.get();
		PERIODICio.right_encoder = rightEnc.get();
		PERIODICio.angle = navx.getAngle();


		// the 1.0 was originally a 10 but is a 1 because it is unessecary to be a 10 as the cancelation of it is no longer present.
		PERIODICio.left_velocity_ticks_per_loop = (int) (leftEnc.getRate()
			/ (1.0 * leftEnc.getDistancePerPulse())); // clicks/second
		PERIODICio.right_velocity_ticks_per_loop = (int) (rightEnc.getRate()
			/ (1.0 * rightEnc.getDistancePerPulse())); // clicks/second
	}

	public void onStart(double t) {/* none */}
	public void onEnd(double t) {/* none */}
	public void disabled(double t) {/* none */}


	/**
	 * Drives the left and right sides of the robot independently.
	 * <p>
	 * <b>DO NOT USE WITH PID.</b>
	 * <p>
	 * The arguments provided are squared to create a more intuitive control
	 * sensitivity.
	 * @param leftSpeed  Percentage speed of left side, from -1 to 1.
	 * @param rightSpeed Percentage speed of right side, from -1 to 1.
	 */
	public void tankDriveTeleop(double leftSpeed, double rightSpeed) {
		differentialDrive.tankDrive(leftSpeed, rightSpeed, true);
	}

	/**
	 * Drives the left and right sides of the robot independently.
	 * <p>
	 * <b>USE WITH PID ONLY.</b>
	 * <p>
	 * The arguments provided are not squared to prevent PID overcompensation.
	 * 
	 * @param leftSpeed  Percentage speed of left side, from -1 to 1.
	 * @param rightSpeed Percentage speed of right side, from -1 to 1.
	 */
	public void tankDriveRaw(double leftSpeed, double rightSpeed) {
		differentialDrive.tankDrive(leftSpeed, rightSpeed, false);
	}

	/**
	 * Sets the forward and turning speeds of the robot independently.
	 * <p>
	 * <b>DO NOT USE WITH PID.</b>
	 * <p>
	 * The arguments provided are squared to create a more intuitive control
	 * sensitivity.
	 * @param forwardSpeed Percentage speed for driving forwards or backwards, from
	 *                     -1 to 1.
	 * @param turnSpeed    Percentage speed for turning, from -1 (left) to 1
	 *                     (right).
	 */
	public void arcadeDriveTeleop(double forwardSpeed, double turnSpeed) {
		differentialDrive.arcadeDrive(forwardSpeed, turnSpeed, true);
	}

	/**
	 * Sets the forward and turning speeds of the robot independently.
	 * <p>
	 * <b>USE WITH PID ONLY.</b>
	 * <p>
	 * The arguments provided are not squared to prevent PID overcompensation.
	 * @param forwardSpeed Percentage speed for driving forwards or backwards, from
	 *                     -1 to 1.
	 * @param turnSpeed    Percentage speed for turning, from -1 (left) to 1
	 *                     (right).
	 */
	public void arcadeDriveRaw(double forwardSpeed, double turnSpeed) {
		differentialDrive.arcadeDrive(forwardSpeed, turnSpeed, false);
	}

	// TODO put the pathfinding code here
	
	/**
	 * Set drive state for which mode of driving should be used.
	 * @param state A state of driving that determines how the drivetrain will move.
	 * @see DriveControlStates
	 */
	public void setDriveState(DriveControlStates state) {
		currentDriveControlState = state;
	}

	/**
	 * Shuts off all drive motors.
	 */
	public void stopDrive() {
		differentialDrive.stopMotor();
	}

	/**
	 * Returns integer value of left encoder (128 clicks per rotation).
	 */
	public int getLeftEncoderClicks() {
		return PERIODICio.left_encoder;
	}

	/**
	 * Returns integer value of right encoder (128 clicks per rotation).
	 */
	public int getRightEncoderClicks() {
		return PERIODICio.right_encoder;
	}

	/**
	 * Returns double value of left encoder in terms of inches.
	 */
	public double getLeftEncoderInches() {
		return PERIODICio.left_encoder * INCHES_DRIVEN_PER_CLICK;
	}

	/**
	 * Returns double value of right encoder in terms of inches.
	 */
	public double getRightEncoderInches() {
		return PERIODICio.right_encoder * INCHES_DRIVEN_PER_CLICK;
	}

	/**
	 * Returns a double value of the speed of the left side in inches/second.
	 */
	public double getLeftLinearVelocity() {
		return PERIODICio.left_velocity_ticks_per_loop * INCHES_DRIVEN_PER_CLICK; // inches/click * clicks/second = inches/second
	}

	/**
	 * Returns a double value of the speed of the right side in inches/second.
	 */
	public double getRightLinearVelocity() {
		return PERIODICio.right_velocity_ticks_per_loop * INCHES_DRIVEN_PER_CLICK; // inches/click * clicks/second = inches/second
	}

	/**
	 * Sets left encoder to zero.
	 */
	public void resetLeftEncoder() {
		leftEnc.reset();
	}

	/**
	 * Sets right encoder to zero.
	 */
	public void resetRightEncoder() {
		rightEnc.reset();
	}

	/**
	 * Resets all drive encoders to 0 clicks.
	 */
	public void resetEncoders() {
		resetLeftEncoder();
		resetRightEncoder();
	}

	/**
	 * Gets current angle (yaw) that the robot is facing in degrees.
	 */
	public double getAngleDegrees() {
		return PERIODICio.angle;
	}

	/**
	 * Gets current angle (yaw) that the robot is facing in radians.
	 */
	public double getAngleRadians() {
		return PERIODICio.angle * (Math.PI / 180.0);
	}

	/**
	 * Sets current robot angle (yaw) as the zero point.
	 */
	public void resetAngle() {
		navx.zeroYaw();
	}

	/**
	 * Converts drive encoder clicks to inches.
	 */
	public static final double clicksToInches(int clicks) {
		return clicks * INCHES_DRIVEN_PER_CLICK;
	}

	/**
	 * Converts inches driven to encoder clicks.
	 */
	public static final int inchesToClicks(double inches) {
		return (int) Math.round(inches / INCHES_DRIVEN_PER_CLICK);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		// builder.addStringProperty("encoders", () -> {
		// 	return (Integer.toString(getLeftEncoderClicks()) + " " + Integer.toString(getRightEncoderClicks()));
		// }, null);
		// builder.addDoubleProperty("left speed", () -> leftGroup.get(), null);
		// builder.addDoubleProperty("right speed", () -> rightGroup.get(), null);
		// builder.addDoubleProperty("angle", () -> getAngleDegrees(), null);
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new DefaultTankDrive());
	}
}