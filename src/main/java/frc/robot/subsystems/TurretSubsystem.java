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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IO;
import frc.robot.RobotMap;
import frc.robot.Vars;


/**
 * A turreting part of the robot that separates the bottom and top of the robot.
 */
public class TurretSubsystem extends SubsystemBase {

  private WPI_TalonSRX turret;

  public static final double CLICKS_PER_DEG = 4100.0 / 360.0; // TODO find this using the gearing of the turret instead of empirically

  /** The maximum amount the turret is allowed to rotate in degrees (+degrees is clockwise) */
  public static final double MAX_ROTATION = 540;
  /** The minimum amount the turret is allowed to rotate in degrees (+degrees is clockwise) */
  public static final double MIN_ROTATION = -540;

  /** The range in degrees the turret can rotate (in degrees). */
  public static final double RANGE_ROTATION = MAX_ROTATION - MIN_ROTATION;

  public TurretSubsystem () {
    turret = new WPI_TalonSRX(RobotMap.ID_TURRET);
    turret.setInverted(Vars.TURRET_REVERSED);
    turret.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
		turret.setSensorPhase(Vars.TURRET_ENCODER_REVERSED);
    turret.config_kP(Constants.PRIMARY_PID, 10, Constants.MS_TIMEOUT);
    
  }
  
  
  static class PERIODICio {

    static int encoder=0;
    
  }


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
      turret.set(ControlMode.Position, toClicks(MIN_ROTATION));
    }
    else if (position>MAX_ROTATION) {
      turret.set(ControlMode.Position, toClicks(MAX_ROTATION));
    }
    else {
      turret.set(ControlMode.Position, toClicks(position));
    }
  }

  /**
   * Rotates the turret to the point closest to the position while being constrained to to the min and max degrees.
   * If it needs to, it will rotate the other direction to get to it's commanded position.
   */
  public void rotateBounded(double position){
    // if the turret is trying to rotate over it's min or max, it should rotate around the other direction
    // this is done by knowing the current count of full rotations and perserving that but bounding the position
    if (position < MIN_ROTATION || position > MAX_ROTATION) {
      int count = (int) ( position/360 ); // count of rotations made
      position = bound(position) + 360 * count;
    }
    System.out.println(position); // TODO Remove debug
    rotateToPosition(position);
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

  /**
   * Takes an angle and bounds it between -180 and 180. (So the angle can only be a half revolution from 0.)
   * @return Same angle within -180 inclusive to 180 exclusive.
   */
  public double bound(double degrees) {
    if (degrees>=0) {
      return (degrees+179)%360-179;
    }
    else {
      return -((-degrees+179)%360-179);
    }
  } 

  /**
   * Gets heading off of robot.
   * Note: The robot's rotation of the turret is the negative of this value.
   */
  public double getRelativeDegrees() {
    return bound(getRotationDegrees());
  }

  // /**
  //  * Get heading off of ground (REQUIRES NAVX!)
  //  */
  // public double getAbsoluteHeading() {

  // }




  @Override
  public void periodic() {
    if (IO.verbose) putDashboard();

    // if (isCentered()) {
    //   resetEncoder();
    // }

    PERIODICio.encoder = turret.getSelectedSensorPosition();
  }

  public void putDashboard() {
    SmartDashboard.putNumber("Turret Gain", turret.getMotorOutputPercent());
    SmartDashboard.putNumber("Turret Encoder", getRotationRaw());
    SmartDashboard.putNumber("Turret Degrees", getRotationDegrees());
  }
}