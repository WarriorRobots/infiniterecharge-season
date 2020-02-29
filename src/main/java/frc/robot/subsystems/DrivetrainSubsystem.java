/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IO;
import frc.robot.RobotMap;
import frc.robot.Vars;

/**
 * Drivetrain subsystem that uses Falcon 500s.
 */
public class DrivetrainSubsystem extends SubsystemBase {

  /**
   * Resolution of the encoders.
   * @see https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-resolution
   */
  private static final int CLICKS_PER_REV = 2048;

  /**
   * Equivilant to 1/Gear Ratio.
   * Use this to convert from 1 rotation of the motor to 1 rotation of the output shaft: input * GEARING = output.
   */
  private static final double GEARING = 12.0/50.0 * 20.0/54.0;

  private static final double WHEEL_DIAMETER = 6;

  private WPI_TalonFX FrontLeft, BackLeft, FrontRight, BackRight;

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

    FrontLeft = new WPI_TalonFX(RobotMap.ID_FRONTLEFT);
    BackLeft = new WPI_TalonFX(RobotMap.ID_BACKLEFT);
    FrontRight = new WPI_TalonFX(RobotMap.ID_FRONTRIGHT);
    BackRight = new WPI_TalonFX(RobotMap.ID_BACKRIGHT);

    FrontLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
    FrontRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
    // Setting the sensor phase is not important as the Differential drive
    // makes the values that come from the right side flipped regardless;
    // a manual flip is located on the periodic
    
    FrontLeft.configStatorCurrentLimit(Vars.DRIVETRAIN_CURRENTLIMIT, Constants.MS_TIMEOUT);
    BackLeft.configStatorCurrentLimit(Vars.DRIVETRAIN_CURRENTLIMIT, Constants.MS_TIMEOUT);
    FrontRight.configStatorCurrentLimit(Vars.DRIVETRAIN_CURRENTLIMIT, Constants.MS_TIMEOUT);
    BackRight.configStatorCurrentLimit(Vars.DRIVETRAIN_CURRENTLIMIT, Constants.MS_TIMEOUT);
    
    LeftGroup = new SpeedControllerGroup(FrontLeft, BackLeft);
    RightGroup = new SpeedControllerGroup(FrontRight, BackRight);
    LeftGroup.setInverted(Vars.LEFT_DRIVE_INVERTED);
    RightGroup.setInverted(Vars.RIGHT_DRIVE_INVERTED);

    drive = new DifferentialDrive(LeftGroup, RightGroup);

    navx = new AHRS(I2C.Port.kMXP);
    
    // Creates odometry class with an initial angle of the current heading of the robot (which should be 0)
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngleDegrees()));
    
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
    LeftGroup.setVoltage(leftVoltage);
    RightGroup.setVoltage(-rightVoltage);
    drive.feed();
  }

  /**
   * Drive with arcade with raw values to go forwards and rotate.
   * 
   * @param x Forwards speed; from -1 to 1.
   * @param z Clockwise speed; from -1 to 1.
   */
  public void arcadedriveRaw(double x, double z) {
    drive.arcadeDrive(x, z, false);
  }

  /**
   * Drive with arcade with squared inputs for human drivers.
   * 
   * @param x Forwards/Backwards speed; from -1 to 1.
   * @param z Clockwise/Counterclockwise speed; from -1 to 1.
   */
  public void arcadedriveSquared(double x, double z) {
    drive.arcadeDrive(x, z, true);
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
   * @return Wheel speeds (in meters/s)
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
    return FrontLeft.getSelectedSensorPosition();
  }

  /**
   * @return the native units of the right encoder.
   */
  public int getRightEnc() {
    return FrontRight.getSelectedSensorPosition() * -1;
    // This is a * -1 because the motor is commanded to go backwards by the differential drive
    // so the motor is still backwards even though we give the differential drive a positive command
  }

  /**
   * @return The inches position of the left encoder.
   */
  public double getLeftPosition() {
    // clicks * rev/clicks * output/input = revs
    // revs * PI * diameter = distance
    return (double) FrontLeft.getSelectedSensorPosition() / CLICKS_PER_REV * GEARING * Math.PI * Vars.WHEEL_DIAMETER;
    // This is a * -1 because the motor is commanded to go backwards by the differential drive
    // so the motor is still backwards even though we give the differential drive a positive command
  }

  /**
   * @return The inches position of the right encoder.
   */
  public double getRightPosition() {
    // clicks * rev/clicks * output/input = revs
    // revs * PI * diameter = distance
    return (double) FrontRight.getSelectedSensorPosition() * -1 / CLICKS_PER_REV * GEARING * Math.PI * Vars.WHEEL_DIAMETER;
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings in inches
   */
  public double getAveragePosition() {
    return (getLeftPosition() + getRightPosition()) / 2.0;
  }

  /**
   * @return The inches/second of the left encoder.
   */
  public double getLeftVelocity() {
    // clicks/100ms * 10(100ms/s) * rev/clicks * output/input = rev/s
    // revs/s * PI * diameter = veloicity (in/s)
    return (double) FrontLeft.getSelectedSensorVelocity() * 10 / CLICKS_PER_REV * GEARING * Math.PI * Vars.WHEEL_DIAMETER;
  }

  /**
   * @return The inches/second of the right encoder.
   */
  public double getRightVelocity() {
    // clicks/100ms * 10(100ms/s) * rev/clicks * output/input = rev/s
    // revs/s * PI * diameter = veloicity (in/s)
    return (double) FrontRight.getSelectedSensorVelocity() * -1 * 10 / CLICKS_PER_REV * GEARING * Math.PI * Vars.WHEEL_DIAMETER;
    // This is a * -1 because the motor is commanded to go backwards by the differential drive
    // so the motor is still backwards even though we give the differential drive a positive command
  }

  /**
	 * Gets yaw angle of the robot. (+Clockwise)
   * Applications that require counterclockwise as postive should take the neagtive of this value.
   * @return angle in degrees.
	 */
	public double getAngleDegrees() {
    return navx.getAngle();
	}

	/**
	 * Gets yaw angle of the robot. (+Clockwise)
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
    FrontLeft.setSelectedSensorPosition(0);
  }
  
  /**
   * Zeros the right drivetrain encoder.
   */
  public void resetRightEnc() {
    FrontRight.setSelectedSensorPosition(0);
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
    SmartDashboard.putNumber("Drivetrain/Navx Degrees", getAngleDegrees());
    SmartDashboard.putNumber("Drivetrain/Navx Radians", getAngleRadians());
    SmartDashboard.putNumber("Drivetrain/Left encoder", getLeftEnc());
    SmartDashboard.putNumber("Drivetrain/Right encoder", getRightEnc());
    SmartDashboard.putNumber("Drivetrain/Left position (in)", getLeftPosition());
    SmartDashboard.putNumber("Drivetrain/Right position (in)", getRightPosition());
    SmartDashboard.putNumber("Drivetrain/Average position (in)", getAveragePosition());
    SmartDashboard.putNumber("Drivetrain/Left veloicity (in*s^-1)", getLeftVelocity());
    SmartDashboard.putNumber("Drivetrain/Right veloicity (in*s^-1)", getRightVelocity());
    SmartDashboard.putNumber("Drivetrain/Odometry X (in)", Units.metersToInches(getPose().getTranslation().getX()));
    SmartDashboard.putNumber("Drivetrain/Odometry Y (in)", Units.metersToInches(getPose().getTranslation().getY()));
  }
}
