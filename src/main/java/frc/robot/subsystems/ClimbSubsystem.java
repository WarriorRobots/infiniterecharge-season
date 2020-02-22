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

public class ClimbSubsystem extends SubsystemBase {
  /**
   * Creates a new Climb.
   */

  /**
   * INFO ABOUT CLIMB:
   * Similar to last years
   * Lot smaller
   * Same system
   * Chain and motors     
   * Pulley system when once motor spins it
   * Like the robot over there ->>>
   * Metal bars push up
   * Hook will also be a part of pulley system and be pushed up
   * 3 stages
   * Goal is to be able to have some sort of ability to stop at certain heights
   * Generator messes with the values
   * Learn about generator and all of it (maybe)	
   */

   /** 
   * QUESTIONS:
   * 3 stages?
   * What is the same system?
   * Act out how it would work
   * What does some of this do? (TO JOSHUA)
   */

  private WPI_TalonSRX pulley;
  private static final int PULLEY_ID = 0; // TODO SET REAL VALUE LATER
  public static final double CLICKS_PER_INCH = 1024.0; // TODO check to see if Climb uses Quaderature Encoder

  public ClimbSubsystem() {
    pulley = new WPI_TalonSRX(PULLEY_ID);
    pulley.setInverted(Vars.CLIMB_PULLEY_INVERTED);
	  pulley.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
	  pulley.setSensorPhase(Vars.CLIMB_ENCODER_INVERTED);
	  pulley.config_kP(Constants.PRIMARY_PID, Vars.CLIMB_P, Constants.MS_TIMEOUT);
  }
  
  /**
   * Moves the climb to the position specified.
   * Has safety built in to avoid crashing the mechanism.
   * @param inches From 0 (the uppermost point) to a negative number (moves the climb legs downward.)
   */ 
  public void moveClimbTo(double inches) {
    if (belowMinimum(inches)) {
      pulley.set(ControlMode.Position, toClicks(Vars.CLIMB_MINIMUM_TARGET));
      System.out.println("CLIMB GOING TO " + inches + ", STOPPING TO PREVENT CRASH");
    } else if (aboveMaximum(inches)) {
      pulley.set(ControlMode.Position, toClicks(Vars.CLIMB_MINIMUM_TARGET));
      System.out.println("CLIMB GOING TO " + inches + ", STOPPING TO PREVENT CRASH");
    } else {
      pulley.set(ControlMode.Position, toClicks(inches));
    }
  }
  
  // TODO establish minimum and maximum with the DESIGNERS
  
  /**
   * Holds the climb legs at the specified position.
   * This has no safeties, so be careful!
   * @param inches Should always be negative.
   */
  public void stabilizeClimb(double inches) {
    pulley.set(ControlMode.Position, toClicks(inches));
  }
    
  public void climbAtPercent(double percent)
  {
    pulley.set(ControlMode.PercentOutput, percent);
  }
  
  /**
   * Drives the winch motor at a constant speed. This has no safeties & can damage
   * the robot, so be careful!
   * 
   * @param speed Percentage speed of the winch, from -1 (down) to 1 (up).
   */
  public void adjustClimbLinear(double speed) {
    double pos = getClimbPosition();
    if (aboveMaximum(pos)) {
      if (speed < 0) {
        pulley.set(speed);
      } else {
        pulley.stopMotor();
        System.out.println("CLIMB GOING TOO FAR UP, STOPPING TO PREVENT CRASH " + pos + " " + speed);
      }
    } else if (belowMinimum(pos)) {
      if (speed > 0) {
        pulley.set(speed);
      } else {
        pulley.stopMotor();
        System.out.println("CLIMB GOING TOO FAR LOW, STOPPING TO PREVENT CRASH " + pos + " " + speed);
      }
    } else {
      pulley.set(speed);
    }
  }
  
  /**
   * Returns true if the specified distance is below the lower bound of motion.
   * <p>True is BAD. Use this in code to avoid crashing the climb.
   * @param inches Any distance measurement related to the climb.
   */
  public boolean belowMinimum(double inches) {
    return inches < Vars.CLIMB_MINIMUM_TARGET;
  }
  
  /**
   * Returns true if the specified distance is above the upper bound of motion.
   * <p>True is BAD. Use this in code to avoid crashing the climb.
   * @param inches Any distance measurement related to the climb.
   */
  public boolean aboveMaximum(double inches) {
    return inches > Vars.CLIMB_MAXIMUM_TARGET;
  }
  
  /**
   * Returns the position of the climb in inches, where 0 is the uppermost point
   * and negative numbers mean a downward extension.
   */
  public double getClimbPosition() {
    return toInches(pulley.getSelectedSensorPosition());
  }
  
  /**
   * Converts encoder clicks to inches.
   * @param clicks Encoder clicks measured from the output axle.
   */
  public double toInches(int clicks) {
    return clicks / CLICKS_PER_INCH;
  }
  
  /**
   * Converts inches to encoder clicks.
   * @param inches Inches traveled, measured from the chain.
   */
  public int toClicks(double inches) {
    return (int) Math.round(inches * CLICKS_PER_INCH);
  }
    
  /**
   * Resets the climb encoder to 0 inches.
   */
  public void resetEncoder() {
    pulley.setSelectedSensorPosition(0);
  }
    
  /**
   * Shuts off the climb pulley motor.
   */
  public void stopClimb() {
    pulley.stopMotor();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  // Thank you Alex for the the Arm and Climb code
}
  