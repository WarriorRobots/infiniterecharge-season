/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO;
import frc.robot.RobotMap;
import frc.robot.Vars;

public class DrivetrainSubsystem extends SubsystemBase {

  /** A Grayhill encoder has {@value} clicks per revolution. */
	public static final int CLICKS_PER_REV = 128;

  /**
   * Equivilant to 1/Gear Ratio.
   * Use this to convert from 1 rotation of the motor to 1 rotation of the output shaft: input * GEARING = output.
   */
  private static final double GEARING = 12.0/50.0 * 20.0/54.0;

  private static final double WHEEL_DIAMETER = 6;

  // 60 in / 3160 pulse
  private static final double DISTANCE_PER_PULSE = 176.0/1157.0;

	private WPI_TalonSRX leftFront, leftMiddle, leftBack, rightFront, rightMiddle, rightBack;

	private Encoder leftEnc, rightEnc;

  private SpeedControllerGroup LeftGroup, RightGroup;

  private DifferentialDrive drive;

  private DifferentialDriveOdometry odometry;

  private AHRS navx;

  /**
	 * The robot travels {@value} inches per encoder click.
	 */
	// Diameter * PI = circumference
	// circumference divided by clicks = distance per click.
	public static final double INCHES_DRIVEN_PER_CLICK = (WHEEL_DIAMETER * Math.PI) / CLICKS_PER_REV;

  public DrivetrainSubsystem() {

    leftFront = new WPI_TalonSRX(RobotMap.LEFT_FRONT_ID);
		leftMiddle = new WPI_TalonSRX(RobotMap.LEFT_MIDDLE_ID);
		leftBack = new WPI_TalonSRX(RobotMap.LEFT_BACK_ID);
		// leftFront.configOpenloopRamp(Vars.DRIVETRAIN_RAMPRATE, Constants.MS_TIMEOUT);
		// leftMiddle.configOpenloopRamp(Vars.DRIVETRAIN_RAMPRATE, Constants.MS_TIMEOUT);
		// leftBack.configOpenloopRamp(Vars.DRIVETRAIN_RAMPRATE, Constants.MS_TIMEOUT);

		rightFront = new WPI_TalonSRX(RobotMap.RIGHT_FRONT_ID);
		rightMiddle = new WPI_TalonSRX(RobotMap.RIGHT_MIDDLE_ID);
		rightBack = new WPI_TalonSRX(RobotMap.RIGHT_BACK_ID);
		// rightFront.configOpenloopRamp(Vars.DRIVETRAIN_RAMPRATE, Constants.MS_TIMEOUT);
		// rightMiddle.configOpenloopRamp(Vars.DRIVETRAIN_RAMPRATE, Constants.MS_TIMEOUT);
		// rightBack.configOpenloopRamp(Vars.DRIVETRAIN_RAMPRATE, Constants.MS_TIMEOUT);

		LeftGroup = new SpeedControllerGroup(leftFront, leftMiddle, leftBack);
		RightGroup = new SpeedControllerGroup(rightFront, rightMiddle, rightBack);

    drive = new DifferentialDrive(LeftGroup, RightGroup);

    navx = new AHRS(I2C.Port.kMXP);
  
		leftEnc = new Encoder(RobotMap.LEFT_ENCODER_PORTA, RobotMap.LEFT_ENCODER_PORTB);
    rightEnc = new Encoder(RobotMap.RIGHT_ENCODER_PORTA, RobotMap.RIGHT_ENCODER_PORTB);
    leftEnc.setReverseDirection(Vars.LEFT_DRIVE_ENCODER_REVERSED);
    rightEnc.setReverseDirection(Vars.RIGHT_DRIVE_ENCODER_REVERSED);
    // distance in inches
    leftEnc.setDistancePerPulse(DISTANCE_PER_PULSE);
    rightEnc.setDistancePerPulse(DISTANCE_PER_PULSE);

    // Creates odometry class with an initial angle of the current heading of the robot (which should be 0)
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(navx.getAngle()));
    
  }

  static class PERIODICio {

    // static double angle = 0; // degrees

    // static int leftEnc = 0; // native units
    // static int rightEnc = 0; // native units

    // static int leftEncVelocity = 0; // native units / 100ms
    // static int rightEncVelocity = 0; // native units / 100ms

  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /**
   * Drive with tankdrive with raw values to the sides for control systems that are non-human.
   * @param left Left speed from -1 to 1.
   * @param right Right speed from -1 to 1.
   */
  public void tankdriveRaw(double left, double right) {
    drive.tankDrive(left, right);
  }

  /**
   * Drive with tankdrive with squared inputs for human drivers.
   * @param left Left speed from -1 to 1.
   * @param right Right speed from -1 to 1.
   */
  public void tankdriveSquared(double left, double right) {
    drive.tankDrive(left, right);
  }

  /**
   * Drive the tankdrive with raw voltage values to give to the motors.
   * @param leftVoltage voltage given to the left side in Volts
   * @param rightVoltage voltage given to the right side in Volts
   */
  public void tankdriveVoltage(double leftVoltage, double rightVoltage) {
    // -rightVoltage to make the right side act reversed
    SmartDashboard.putNumber("Left Output", leftVoltage);
    SmartDashboard.putNumber("Right Output", rightVoltage);
    LeftGroup.setVoltage(leftVoltage);
    RightGroup.setVoltage(-rightVoltage);
    drive.feed();
  }

  /**
   * Return estimated pose of the robot.
   * 
   * @return current pose (in meters)
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   * 
   * @return Wheel speeds (in meters)
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      Units.inchesToMeters(getLeftVelocity()),
      Units.inchesToMeters(getRightVelocity())
    );
  }

  /**
   * @return the native units of the left encoder.
   */
  public int getLeftEnc() {
    return leftEnc.get();
  }

  /**
   * @return the native units of the right encoder.
   */
  public int getRightEnc() {
    return rightEnc.get();
  }

  /**
   * @return The inches of the left encoder.
   */
  public double getLeftPosition() {
    return leftEnc.getDistance();
  }

  /**
   * @return The inches position of the right encoder.
   */
  public double getRightPosition() {
    return rightEnc.getDistance();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings in inches
   */
  public double getAverageEncoderDistance() {
    return (getLeftPosition() + getRightEnc()) / 2.0;
  }

  /**
   * @return The inches/second of the left encoder.
   */
  public double getLeftVelocity() {
    return leftEnc.getRate();
  }

  /**
   * @return The inches/second of the right encoder.
   */
  public double getRightVelocity() {
    // clicks/100ms * 10(100ms/s) * rev/clicks * output/input = rev/s
    // revs/s * PI * diameter = veloicity (in/s)
    return rightEnc.getRate();
  }

  /**
	 * Gets yaw angle of the robot.
   * @return angle in degrees.
	 */
	public double getAngleDegrees() {
    return navx.getAngle();
	}

	/**
	 * Gets yaw angle of the robot.
   * @return angle in radians.
	 */
	public double getAngleRadians() {
		return Units.degreesToRadians(getAngleDegrees());
  }

  /**
   * Get the turn rate of the robot
   * @return turn rate in degrees/s
   */
  public double getTurnRate() {
    return navx.getRate();
  }

  /**
   * Zeros the drivetrain yaw angle.
   */
  public void resetAngle() {
    navx.zeroYaw();
  }
  
  /**
   * Zeros the left drivetrain encoder.
   */
  public void resetLeftEnc() {
    leftEnc.reset();
  }
  
  /**
   * Zeros the right drivetrain encoder.
   */
  public void resetRightEnc() {
    rightEnc.reset();
  }

  /**
   * Zeros the drivetrain encoders.
   */
  public void resetEnc() {
    resetLeftEnc();
    resetRightEnc();
  }

  /**
   * Resets the odometry of the robot
   * @param pose of the robot when it is reset (can be a default pose to )
   */
  public void resetOdometry() {
    reset();
    odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(getAngleDegrees()));
  }
  
  /**
   * Zeros ALL the sensors affiliated with the drivetrain.
   */
  public void reset() {
    resetAngle();
    resetEnc();
  }

  /**
   * Stops the drivetrain.
   */
  public void stop() {
    drive.stopMotor();
  }

  @Override
  public void periodic() {
    
    if (IO.verbose) putDashboard();

    odometry.update(
      Rotation2d.fromDegrees(getAngleDegrees()),
      Units.inchesToMeters(getLeftPosition()),
      Units.inchesToMeters(getRightPosition())
    );
  }

  /**
   * Puts information about this subsystem on the dashboard.
   */
  public void putDashboard() {
    SmartDashboard.putNumber("Navx Degrees", getAngleDegrees());
    SmartDashboard.putNumber("Navx Radians", getAngleRadians());
    SmartDashboard.putNumber("Left encoder", getLeftEnc());
    SmartDashboard.putNumber("Right encoder", getRightEnc());
    SmartDashboard.putNumber("Left position (in)", getLeftPosition());
    SmartDashboard.putNumber("Right position (in)", getRightPosition());
    SmartDashboard.putNumber("Left veloicity (in/s)", getLeftVelocity());
    SmartDashboard.putNumber("Right veloicity (in/s)", getRightVelocity());

    SmartDashboard.putNumber("Odometry X", Units.metersToInches(getPose().getTranslation().getX()));
    SmartDashboard.putNumber("Odometry Y", Units.metersToInches(getPose().getTranslation().getY()));
  }
}
