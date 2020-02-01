/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Vars;

/**
 * Contains the a single wheel shooter with a PID to command the velocity of the shooter.
 */
public class ShooterSubsystem extends SubsystemBase {

  private static final int ID_SHOOTER = 0; // TODO GO BACK LATER TO FIX ID

  private WPI_TalonSRX shooter;

  /** Number of encoder clicks per every revolution of the encoder */
  static final int CLICKS_PER_REV = 4096; // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-resolution
  /** Typical motor output as percent */
  static final double ESTIMATED_VOLTAGE = .85; 
  /** Velocity of shooter in native units per 100ms at typical motor output (at the encoder) */
  static final int NATIVE_ESTIMATED_VELOCITY = 25000; 
  /**
   * The reciprocal of the gear ratio.
   * This is so cancelation can occur to calculate the speed on either side of the ratio.
   * ex. OUTSIDE speed * IN/OUT = INSIDE speed.
  */
  static final double OUT_IN = 22.0/16.0;

  /**
   * Instantiates new subsystem; make ONLY ONE.
	 * <p>
	 * <code> public static final ShooterSubsystem shooter = new
	 * ShooterSubsystem();
	 */
  public ShooterSubsystem()
  {
    shooter = new WPI_TalonSRX(ID_SHOOTER);
    shooter.setInverted(Vars.SHOOTER_REVERSED);
    shooter.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
    shooter.setSensorPhase(Vars.SHOOTER_ENCODER_REVERSED);
    shooter.config_kF(Constants.PRIMARY_PID, ESTIMATED_VOLTAGE*1023/NATIVE_ESTIMATED_VELOCITY, Constants.MS_TIMEOUT); // https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html#calculating-velocity-feed-forward-gain-kf
    shooter.config_kP(Constants.PRIMARY_PID, Vars.SHOOTER_KP, Constants.MS_TIMEOUT);
   }

  /**
   * Run the shooter motor at a percent from -1 to 1.
   * @param voltage Percent from -1 to 1.
   */
  public void setVoltage(double voltage)
  {
    shooter.set(ControlMode.PercentOutput, voltage);
  }

  /**
   * Run the shooter motor at an RPM.
   * @param rpm RPM of the shooter flywheel. (Not the motor or encoder.)
   */
  public void setRPM(double rpm)
  {
    shooter.set(ControlMode.Velocity, toNative(rpm/OUT_IN));
  }

  /**
   * @return The percent the talon is commanding to the motor.
   */
  public double getGain()
  {
    return shooter.getMotorOutputPercent();
  }

  /**
   * @return The RPM of the shooter flywheel. (Not the motor or encoder.)
   */
  public double getRPM()
  {
    return toRPM(shooter.getSelectedSensorVelocity()*OUT_IN);
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
   * Converts from RPM to native units per 100ms.
   * @param rpm RPM
   * @return Native units / 100ms
   */
  public static double toNative(double rpm)
  { 
    return ((rpm / 600) * CLICKS_PER_REV);
  }  
}