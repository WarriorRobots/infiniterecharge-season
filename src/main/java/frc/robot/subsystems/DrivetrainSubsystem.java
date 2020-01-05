package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.Constants;
import frc.robot.QuickAccessVars;
import frc.robot.commands.drive.DefaultTankDrive;

/**
 * Contains the drivetrain, the encoders for the left and right wheels, and the NavX gyroscope.
 */
public class DrivetrainSubsystem extends IterSubsystem {

	private static final int LEFT_FRONT_ID = 1;
	private static final int LEFT_MIDDLE_ID = 2;
	private static final int LEFT_BACK_ID = 3;
	private static final int RIGHT_FRONT_ID = 4;
	private static final int RIGHT_MIDDLE_ID = 5;
	private static final int RIGHT_BACK_ID = 6;

	private static final int LEFT_ENCODER_PORTA = 0;
	private static final int LEFT_ENCODER_PORTB = 1;
	private static final int RIGHT_ENCODER_PORTA = 2;
	private static final int RIGHT_ENCODER_PORTB = 3;

	private WPI_TalonSRX leftFront, leftMiddle, leftBack, rightFront, rightMiddle, rightBack;
	private SpeedControllerGroup leftGroup, rightGroup;
	private DifferentialDrive differentialDrive;

	private Encoder leftEnc, rightEnc;
	private AHRS navx;

	// private Path currentPath = null;
	// private PathFollower pathFollower;
	private DriveControlStates currentDriveControlState;

	/** A Grayhill encoder has {@value} clicks per revolution. */
	public static final int CLICKS_PER_REV = 128;

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

	private static int left_velocity_ticks_per_loop=0; // clicks/second
	private static int right_velocity_ticks_per_loop=0; // clicks/second

	/**
	 * Instantiates new subsystem; make ONLY ONE.
	 * <p>
	 * <code> public static final DrivetrainSubsystem drivetrain = new
	 * DrivetrainSubsystem();
	 */
	public DrivetrainSubsystem() {
		leftFront = new WPI_TalonSRX(LEFT_FRONT_ID);
		leftMiddle = new WPI_TalonSRX(LEFT_MIDDLE_ID);
		leftBack = new WPI_TalonSRX(LEFT_BACK_ID);
		leftFront.configOpenloopRamp(QuickAccessVars.DRIVETRAIN_RAMPRATE, Constants.TIMEOUT_MS);
		leftMiddle.configOpenloopRamp(QuickAccessVars.DRIVETRAIN_RAMPRATE, Constants.TIMEOUT_MS);
		leftBack.configOpenloopRamp(QuickAccessVars.DRIVETRAIN_RAMPRATE, Constants.TIMEOUT_MS);

		rightFront = new WPI_TalonSRX(RIGHT_FRONT_ID);
		rightMiddle = new WPI_TalonSRX(RIGHT_MIDDLE_ID);
		rightBack = new WPI_TalonSRX(RIGHT_BACK_ID);
		rightFront.configOpenloopRamp(QuickAccessVars.DRIVETRAIN_RAMPRATE, Constants.TIMEOUT_MS);
		rightMiddle.configOpenloopRamp(QuickAccessVars.DRIVETRAIN_RAMPRATE, Constants.TIMEOUT_MS);
		rightBack.configOpenloopRamp(QuickAccessVars.DRIVETRAIN_RAMPRATE, Constants.TIMEOUT_MS);

		leftGroup = new SpeedControllerGroup(leftFront, leftMiddle, leftBack);
		rightGroup = new SpeedControllerGroup(rightFront, rightMiddle, rightBack);
		leftGroup.setInverted(QuickAccessVars.LEFT_SIDE_REVERSED);
		rightGroup.setInverted(QuickAccessVars.RIGHT_SIDE_REVERSED);

		differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
		differentialDrive.setSafetyEnabled(false);

		// if NavX is missing, this code will handle errors and prevent a crash
		try {
			navx = new AHRS(I2C.Port.kMXP);
		} catch (Exception ex) {
			DriverStation.reportError(ex.getMessage(), true);
		}

		leftEnc = new Encoder(LEFT_ENCODER_PORTA, LEFT_ENCODER_PORTB);
		rightEnc = new Encoder(RIGHT_ENCODER_PORTA, RIGHT_ENCODER_PORTB);

		leftEnc.setReverseDirection(QuickAccessVars.LEFT_SIDE_ENCODER_REVERSED);
		rightEnc.setReverseDirection(QuickAccessVars.RIGHT_SIDE_ENCODER_REVERSED);

		// Drive state starts out as Open loop, following driver commands or voltage commands
		currentDriveControlState = DriveControlStates.OPEN_LOOP;
	}

	/**
	 * Runs on the loop of the robot
	 * @param timestamp Timestamp of the robot so it knows the match time.
	 */
	public void onLoop(double timestamp) {
		synchronized (DrivetrainSubsystem.this) {
			if (currentDriveControlState == DriveControlStates.PATH_FOLLOWING && currentPath != null) {
				updatePathFollower(timestamp);
			}
		}

		// the 1.0 was originally a 10 but is a 1 because it is unessecary to be a 10 as the cancelation of it is no longer present.
		left_velocity_ticks_per_loop = (int) (leftEnc.getRate()
                / (1.0 * leftEnc.getDistancePerPulse())); // clicks/second
        right_velocity_ticks_per_loop = (int) (rightEnc.getRate()
                / (1.0 * rightEnc.getDistancePerPulse())); // clicks/second
	}

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
	
	public void setOpen() {
		currentDriveControlState = DriveControlStates.OPEN_LOOP;
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
		return leftEnc.get();
	}

	/**
	 * Returns integer value of right encoder (128 clicks per rotation).
	 */
	public int getRightEncoderClicks() {
		return rightEnc.get();
	}

	/**
	 * Returns double value of left encoder in terms of inches.
	 */
	public double getLeftEncoderInches() {
		return leftEnc.get() * INCHES_DRIVEN_PER_CLICK;
	}

	/**
	 * Returns double value of right encoder in terms of inches.
	 */
	public double getRightEncoderInches() {
		return rightEnc.get() * INCHES_DRIVEN_PER_CLICK;
	}

	/**
	 * Returns a double value of the speed of the left side in inches/second.
	 */
	public double getLeftLinearVelocity() {
		return INCHES_DRIVEN_PER_CLICK * left_velocity_ticks_per_loop; // inches/click * clicks/second = inches/second
	}

	/**
	 * Returns a double value of the speed of the right side in inches/second.
	 */
	public double getRightLinearVelocity() {
		return INCHES_DRIVEN_PER_CLICK * right_velocity_ticks_per_loop; // inches/click * clicks/second = inches/second
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
		return navx.getAngle();
	}

	/**
	 * Gets current angle (yaw) that the robot is facing in radians.
	 */
	public double getAngleRadians() {
		return navx.getAngle() * (Math.PI / 180.0);
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

	private enum DriveControlStates {
		OPEN_LOOP, // following controls from driver or velocities
		PATH_FOLLOWING, // following controls from path
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new DefaultTankDrive());
	}
}