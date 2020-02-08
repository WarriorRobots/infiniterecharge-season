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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IO;
import frc.robot.RobotMap;
import frc.robot.Vars;

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

  private AHRS navx;

  public DrivetrainSubsystem() {

    FrontLeft = new WPI_TalonFX(RobotMap.ID_FRONTLEFT);
    BackLeft = new WPI_TalonFX(RobotMap.ID_BACKLEFT);
    FrontRight = new WPI_TalonFX(RobotMap.ID_FRONTRIGHT);
    BackRight = new WPI_TalonFX(RobotMap.ID_BACKRIGHT);

    // HELP ME JOSHUA 
    // AHHHHHHH
    // NO PRIMARY_PID
    //FrontLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
    //FrontRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
    // Setting the sensor phase is not important as the Differential drive
    // makes the values that come from the right side flipped regardless;
    // a manual flip is located on the periodic

    LeftGroup = new SpeedControllerGroup(FrontLeft, BackLeft);
    RightGroup = new SpeedControllerGroup(FrontRight, BackRight);
    LeftGroup.setInverted(Vars.LEFT_DRIVE_INVERTED);
    RightGroup.setInverted(Vars.RIGHT_DRIVE_INVERTED);

    drive = new DifferentialDrive(LeftGroup, RightGroup);

    // if NavX is missing, this code will handle errors and prevent a crash
		try {
      navx = new AHRS(I2C.Port.kMXP);
		} catch (Exception e) {
			DriverStation.reportError(e.getMessage(), true);
    }
    
  }

  static class PERIODICio {

    static double angle = 0; // degrees

    static int leftEnc = 0; // native units
    static int rightEnc = 0; // native units

    static int leftEncVelocity = 0; // native units / 100ms
    static int rightEncVelocity = 0; // native units / 100ms

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
   * @return the native units of the left encoder.
   */
  public int getLeftEnc() {
    return PERIODICio.leftEnc;
  }

  /**
   * @return the native units of the right encoder.
   */
  public int getRightEnc() {
    return PERIODICio.rightEnc;
  }

  /**
   * @return The inches of the left encoder.
   */
  public double getLeftPosition() {
    // clicks * rev/clicks * output/input = revs
    // revs * PI * diameter = distance
    return (double) PERIODICio.leftEnc / CLICKS_PER_REV * GEARING * Math.PI * WHEEL_DIAMETER;
  }

  /**
   * @return The inches position of the right encoder.
   */
  public double getRightPosition() {
    // clicks * rev/clicks * output/input = revs
    // revs * PI * diameter = distance
    return (double) PERIODICio.rightEnc / CLICKS_PER_REV * GEARING * Math.PI * WHEEL_DIAMETER;
  }

  /**
   * @return The inches/second of the left encoder.
   */
  public double getLeftVelocity() {
    // clicks/100ms * 10(100ms/s) * rev/clicks * output/input = rev/s
    // revs/s * PI * diameter = veloicity (in/s)
    return (double) PERIODICio.leftEncVelocity * 10 / CLICKS_PER_REV * GEARING * Math.PI * WHEEL_DIAMETER;
  }

  /**
   * @return The inches/second of the right encoder.
   */
  public double getRightVelocity() {
    // clicks/100ms * 10(100ms/s) * rev/clicks * output/input = rev/s
    // revs/s * PI * diameter = veloicity (in/s)
    return (double) PERIODICio.rightEncVelocity * 10 / CLICKS_PER_REV * GEARING * Math.PI * WHEEL_DIAMETER;
  }

  /**
	 * Gets yaw angle of the robot.
   * @return angle in degrees.
	 */
	public double getAngleDegrees() {
		return PERIODICio.angle;
	}

	/**
	 * Gets yaw angle of the robot.
   * @return angle in radians.
	 */
	public double getAngleRadians() {
		return PERIODICio.angle * (Math.PI / 180.0);
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
   * Zeros ALL the sensors affiliated with the drivetrain.
   */
  public void reset() {
    resetAngle();
    resetEnc();
  }

  @Override
  public void periodic() {
    
    if (IO.verbose) putDashboard();
    PERIODICio.angle = navx.getAngle();
    PERIODICio.leftEnc = FrontLeft.getSelectedSensorPosition();
    PERIODICio.rightEnc = FrontRight.getSelectedSensorPosition() * -1;
    // This is a * -1 because the motor is commanded to go backwards by the differential drive
    // so the motor is still backwards even though we give the differential drive a positive command
    PERIODICio.leftEncVelocity = FrontLeft.getSelectedSensorVelocity();
    PERIODICio.rightEncVelocity = FrontRight.getSelectedSensorVelocity() * -1;
    // see above

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
  }
}
