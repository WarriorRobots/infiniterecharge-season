/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.QuickAccessVars;

/**
 * Add your docs here.
 */
public class ShooterSubSystem extends Subsystem {

  private static final int ID_SHOOTER = 0; // TODO GO BACK LATER TO FIX ID
  

  private WPI_TalonSRX shooter;

  /** number of encoder clicks per every revolution of the encoder */
  static final int CLICKS_PER_REV = 4096; // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-resolution
  /** typical motor output as percent */
  static final double ESTIMATED_VOLTAGE = .85; 
  /** native units per 100ms at typical motor output */
  static final int NATIVE_ESTIMATED_VELOCITY = 25000; 
  /** gear ratio so that the units cancel the operations cancel */
  static final double GEAROUT_GEARIN = 22.0/16.0;
  // WHEEL = RATIO * ENCODER
  // OUTSIDE * INSIDE/OUTSIDE = INSIDE

  // constructor used to initialize objects
  public ShooterSubSystem()
  {
    shooter = new WPI_TalonSRX(ID_SHOOTER);
    shooter.setInverted(QuickAccessVars.SHOOTER_REVERSED);
    shooter.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.PID_ID, Constants.TIMEOUT_MS);
    shooter.setSensorPhase(QuickAccessVars.SHOOTER_ENCODER_REVERSED);
    shooter.config_kF(Constants.PID_ID, ESTIMATED_VOLTAGE*1023/NATIVE_ESTIMATED_VELOCITY, Constants.TIMEOUT_MS); // https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html#calculating-velocity-feed-forward-gain-kf
    shooter.config_kP(Constants.PID_ID, QuickAccessVars.SHOOTER_KP, Constants.TIMEOUT_MS);
   }

  /** set the motor to a voltage from -1 to 1 */
  public void setVoltage(double voltage)
  {
    shooter.set(ControlMode.PercentOutput, voltage);
  }

  public void setRPM(double rpm)
  {
    shooter.set(ControlMode.Velocity, toNative(rpm/GEAROUT_GEARIN));
  }

  public double getGain()
  {
    // return gain
    // problem is what is gain?
    // do research (Joshua doesn't know lol)
    return shooter.getMotorOutputPercent();
  }

  /** the rpm of the wheel */
  public double getRPM()
  {
    
    return toRPM(shooter.getSelectedSensorVelocity()*GEAROUT_GEARIN);

  }

  public static double toRPM(double native_units)
  { 
    return ((native_units * 600) / CLICKS_PER_REV);
  }
  
  public static double toNative(double rpm)
  { 
    return ((rpm / 600) * CLICKS_PER_REV);
  }

  @Override
  public void initDefaultCommand() {
    
  }



}
