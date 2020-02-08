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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * A turreting part of the robot that separates the bottom and top of the robot.
 */
public class TurretSubsystem extends SubsystemBase {

  private static final int ID_TURRET = 0; // TODO Fix id
  private static final int LIMIT_SWITCH_PORT = 0; // TODO Fix port #

  private WPI_TalonSRX turret;
  private DigitalInput limitSwitch;

  public static final double CLICKS_PER_DEG = 128; // TODO Find Clicks per rev

  /** The maximum amount the turret is allowed to rotate in degrees (+degrees is clockwise) */
  public static final double MAX_ROTATION = 180;
  /** The minimum amount the turret is allowed to rotate in degrees (+degrees is clockwise) */
  public static final double MIN_ROTATION = -180;

  /** The range in degrees the turret can rotate (in degrees). */
  public static final double RANGE_ROTATION = MAX_ROTATION - MIN_ROTATION;

  public TurretSubsystem () {
    turret = new WPI_TalonSRX(ID_TURRET);
    turret.setInverted(false); // TODO Check if it is reversed
    turret.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10); // 0 PID and 10MS come from other robot
		turret.setSensorPhase(false); // TODO Check if the encoder is flipped
    turret.config_kP(0, 0, 10); // PID, kP, MS TODO set kP for turret
    
    limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
  }
  
  
  static class PERIODICio {

    static int encoder=0;
    
  }

  public void periodic(double t) {
    if (isCentered()) {
      resetEncoder();
    }

    PERIODICio.encoder = turret.getSelectedSensorPosition();
  }
  
  public void onStart(double t) {/* none */}
  public void onLoop(double t) {/* none */}
  public void onEnd(double t) {/* none */}
  public void disabled(double t) {/* none */}


  /** 
   * Give the turret a voltage to rotate.
   * WARNING for testing, should have safety for normal use
   * 
   * @param voltage Decimal percentage from -1 to 1. 1 is clockwise.
   */
  public void rotateNoSafety(double voltage) {
    turret.set(voltage);
  }

  /** 
   * Give the turret a voltage to rotate.
   * 
   * @param voltage Decimal percentage from -1 to 1. 1 is clockwise.
   */
  public void rotate(double voltage) {
    if (voltage>0 && getRotationDegrees()<MAX_ROTATION) {turret.set(voltage);} // Clockwise
    else if (voltage<0 && getRotationDegrees()>MIN_ROTATION) {turret.set(voltage);} // Counterclockwise
    else {turret.stopMotor();}
  }

  /**
   * Rotates to a position given as a heading relative to the robot
   * 
   * @param position Double representing where the robot is rotated in degrees
   */
  public void rotateToPosition(double position) {
    if (position<MIN_ROTATION) {
      turret.set(ControlMode.Position, MIN_ROTATION);
    }
    else if (position>MAX_ROTATION) {
      turret.set(ControlMode.Position, MAX_ROTATION);
    }
    else {
      turret.set(ControlMode.Position, toClicks(position));
    }
  }

  /**
   * Stops the turret motor.
   */
  public void stop() {
    turret.stopMotor();
  }

  /** 
   * Gets the rotation of the turret based on encoder value
   * 
   * @return Encoder value of turret. (+value is clockwise)
   */
  public double getRotationRaw() {
    return PERIODICio.encoder;
  }

  /** 
   * Gets the rotation of the turret based on encoder value
   * 
   * @return Degree rotation of turret. (+degree is clockwise)
   */
  public double getRotationDegrees() {
    return PERIODICio.encoder/CLICKS_PER_DEG;
  }

  /**
	 * Zeroes out the turret encoder.
	 */
	private void resetEncoder() {
		turret.setSelectedSensorPosition(0);
  }
  
  /**
	 * Returns true if the limit switch at the center of the turret is being triggered.
	 */
	public boolean isCentered() {
		return !limitSwitch.get(); // flipped because sensor reads true if there is NO magnet
  }
  
  /** 
   * Converts betweens degrees and encoder clicks.
   */
  public int toClicks(double degrees) {
    return (int) Math.round(degrees*CLICKS_PER_DEG);
  }

  /** 
   * Converts betweens encoder clicks and degrees.
   */
  public double toDegrees(double clicks) {
    return clicks/CLICKS_PER_DEG;
  }

  // Gets heading off of robot (REQUIRES NAVX!)
  //public double getRelativeHeading() {}

  // Get heading off of ground (REQUIRES NAVX!)
  //public double getAbsoluteHeading() {}



}