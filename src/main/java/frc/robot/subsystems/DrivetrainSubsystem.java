/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IO;

public class DrivetrainSubsystem extends SubsystemBase {
  
  private static final int ID_FRONTLEFT = 14;
  private static final int ID_BACKLEFT = 15;
  private static final int ID_FRONTRIGHT = 1;
  private static final int ID_BACKRIGHT = 0;

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

  public DrivetrainSubsystem() {

    FrontLeft = new WPI_TalonFX(ID_FRONTLEFT);
    BackLeft = new WPI_TalonFX(ID_BACKLEFT);
    FrontRight = new WPI_TalonFX(ID_FRONTRIGHT);
    BackRight = new WPI_TalonFX(ID_BACKRIGHT);

    FrontLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
    FrontRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
    FrontLeft.setSensorPhase(false);
    FrontRight.setSensorPhase(true);

    LeftGroup = new SpeedControllerGroup(FrontLeft, BackLeft);
    RightGroup = new SpeedControllerGroup(FrontRight, BackRight);
    LeftGroup.setInverted(false);
    RightGroup.setInverted(false);

    drive = new DifferentialDrive(LeftGroup, RightGroup);
  }

  static class PERIODICio {

    static int leftEnc = 0; // native units
    static int rightEnc = 0; //native units

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (IO.verbose) putDashboard();
    PERIODICio.leftEnc = FrontLeft.getSelectedSensorPosition();
    PERIODICio.rightEnc = FrontRight.getSelectedSensorPosition();
    PERIODICio.leftEncVelocity = FrontLeft.getSelectedSensorVelocity();
    PERIODICio.rightEncVelocity = FrontRight.getSelectedSensorVelocity();

  }

  /**
   * Puts information about this subsystem on the dashboard.
   */
  public void putDashboard() {
    SmartDashboard.putNumber("Left encoder", getLeftEnc());
    SmartDashboard.putNumber("Right encoder", getRightEnc());
    SmartDashboard.putNumber("Left position (in)", getLeftPosition());
    SmartDashboard.putNumber("Right position (in)", getRightPosition());
    SmartDashboard.putNumber("Left veloicity (in/s)", getLeftVelocity());
    SmartDashboard.putNumber("Right veloicity (in/s)", getRightVelocity());
  }
}
