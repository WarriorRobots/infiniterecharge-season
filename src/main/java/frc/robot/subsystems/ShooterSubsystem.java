/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IO;
import frc.robot.RobotMap;
import frc.robot.Vars;

/**
 * Contains the a single wheel shooter with a PID to command the velocity of the shooter.
 */
public class ShooterSubsystem extends SubsystemBase {

  private WPI_TalonFX shooter_left;
  private WPI_TalonFX slave_right;

  /** Number of encoder clicks per every revolution of the encoder */
  static final int CLICKS_PER_REV = 4096; // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-resolution
  /** Typical motor output as percent */
  static final double ESTIMATED_VOLTAGE = .57;
  /** Velocity of shooter in native units per 100ms at typical motor output (at the encoder) */
  static final int NATIVE_ESTIMATED_VELOCITY = 12467;

  /**
   * Instantiates new subsystem; make ONLY ONE.
	 * <p>
	 * <code> public static final ShooterSubsystem shooter = new
	 * ShooterSubsystem();
	 */
  public ShooterSubsystem()
  {
    shooter_left = new WPI_TalonFX(RobotMap.ID_SHOOTER_LEFT);
    
    shooter_left.setInverted(Vars.SHOOTER_LEFT_REVERSED);
    shooter_left.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
    shooter_left.config_kF(Constants.PRIMARY_PID, ESTIMATED_VOLTAGE*1023/NATIVE_ESTIMATED_VELOCITY, Constants.MS_TIMEOUT); // https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html#calculating-velocity-feed-forward-gain-kf
    shooter_left.config_kP(Constants.PRIMARY_PID, Vars.SHOOTER_KP, Constants.MS_TIMEOUT);
    
    slave_right = new WPI_TalonFX(RobotMap.ID_SHOOTER_RIGHT);
    slave_right.follow(shooter_left);
    slave_right.setInverted(Vars.SHOOTER_RIGHT_REVERSED);

    SmartDashboard.putNumber("Shooter/RPM Command", Vars.SHOOTER_DEFAULT);
    SmartDashboard.putNumber("Shooter/Percentage Command", ESTIMATED_VOLTAGE);
   }

  /**
   * Run the shooter motor at a percent from -1 to 1.
   * @param voltage Percent from -1 to 1.
   */
  public void setVoltage(double voltage)
  {
    shooter_left.set(ControlMode.PercentOutput, voltage);
  }

  /**
   * Run the shooter motor at an RPM.
   * @param rpm RPM of the shooter flywheel. (Not the motor or encoder.)
   */
  public void setRPM(double rpm)
  {
    shooter_left.set(ControlMode.Velocity, toNative(rpm));
  }

  /**
   * @return The percent the talon is commanding to the motor.
   */
  public double getGain()
  {
    return shooter_left.getMotorOutputPercent();
  }

  /**
   * @return The RPM of the shooter flywheel. (Not the motor or encoder.)
   */
  public double getRPM()
  {
    // (native / 100ms) * (600ms / m) * (rev/native) = rev / m
    return toRPM(shooter_left.getSelectedSensorVelocity());
  }

  /**
   * @return the raw encoder value.
   */
  public int getEnc() {
    return shooter_left.getSelectedSensorPosition();
  }

  /**
   * @return the velocity of the motor in 
   */
  public double getEncVelocity() {
    return shooter_left.getSelectedSensorVelocity();
  }

  /**
   * Converts from native units per 100ms to RPM.
   * @param native_units Native units / 100ms
   * @return RPM
   */
  public static double toRPM(double native_units)
  { 
    return ((native_units * 600) / CLICKS_PER_REV);
  }

  /**
   * @return The value from the dashboard for how fast the shooter should be going in RPM.
   */
  public double getCommandedRPM() {
    return SmartDashboard.getNumber("Shooter/RPM Command", Vars.SHOOTER_DEFAULT);
  }

  /**
   * @return The value from the dashboard for how fast the shooter should be based on a percentage.
   */
  public double getCommandedPercent() {
    return SmartDashboard.getNumber("Shooter/Percentage Command", 0);
  }
  
  /**
   * Converts from RPM to native units per 100ms.
   * @param rpm RPM
   * @return Native units / 100ms
   */
  public static double toNative(double rpm)
  { 
    return ((rpm / 600) * CLICKS_PER_REV);
  }

  public void stop() {
    shooter_left.stopMotor();
  }

  @Override
  public void periodic() {
    if (IO.verbose) putDashboard();
  }

  public void putDashboard() {
    SmartDashboard.putNumber("Shooter/Gain", getGain());
    SmartDashboard.putNumber("Shooter/Encoder", getEnc());
    SmartDashboard.putNumber("Shooter/Native units per 100ms", getEncVelocity());
    SmartDashboard.putNumber("Shooter/RPM", getRPM());
  }
}