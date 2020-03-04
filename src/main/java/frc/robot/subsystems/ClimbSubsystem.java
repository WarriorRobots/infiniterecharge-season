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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.IO;
import frc.robot.Vars;

public class ClimbSubsystem extends SubsystemBase {
  // motors use clicks
  private WPI_TalonSRX winch;
  private static final int WINCH_ID = 0; // TODO SET REAL VALUE LATER
  public static final double CLICKS_PER_INCH = 1024.0; // TODO check to see if Climb uses Quaderature Encoder
  private static final int PNEUMATIC_FORWARD = 1;
  private static final int PNEUMATIC_BACKWARD = 0;
  private DoubleSolenoid climbLock; 

  /**
   * Creates a new Climb.
   */
  public ClimbSubsystem() {
    climbLock = new DoubleSolenoid(Constants.PCM_1, PNEUMATIC_FORWARD, PNEUMATIC_BACKWARD);
    winch = new WPI_TalonSRX(WINCH_ID);
    winch.setInverted(Vars.CLIMB_CHAIN_INVERTED);
	  winch.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
	  winch.setSensorPhase(Vars.CLIMB_ENCODER_INVERTED);
    winch.config_kP(Constants.PRIMARY_PID, Vars.CLIMB_P, Constants.MS_TIMEOUT);
    
  }
  
  /**
   * Moves the climb to the position specified.
   * Has safety built in to avoid crashing the mechanism.
   * @param inches From 0 (the uppermost point) to a negative number (moves the climb legs downward.)
   */ 
  public void moveClimbTo(double inches) { // position
    if (belowMinimum(inches)) {
      winch.set(ControlMode.Position, toClicks(Vars.CLIMB_MINIMUM_TARGET));
      System.out.println("CLIMB GOING TO " + inches + ", STOPPING TO PREVENT CRASH");
    } else if (aboveMaximum(inches)) {
      winch.set(ControlMode.Position, toClicks(Vars.CLIMB_MINIMUM_TARGET));
      System.out.println("CLIMB GOING TO " + inches + ", STOPPING TO PREVENT CRASH");
    } else {
      winch.set(ControlMode.Position, toClicks(inches));
    }
  }
  
  // TODO establish minimum and maximum with the DESIGNERS
  
  /**
   * Holds the climb legs at the specified position.
   * This has no safeties, so be careful!
   * @param inches Should always be negative.
   */
  public void stabilizeClimb(double inches) {
    winch.set(ControlMode.Position, toClicks(inches));
  }
    
  public void climbAtPercent(double percent)
  {
    winch.set(ControlMode.PercentOutput, percent);
  }
  
  /**
   * Drives the winch motor at a constant speed. This has no safeties & can damage
   * the robot, so be careful!
   * 
   * @param speed Percentage speed of the winch, from -1 (down) to 1 (up).
   */
  public void adjustClimbLinear(double speed) { // linear
    double pos = getClimbPosition();
    if (aboveMaximum(pos)) {
      if (speed < 0) {
        winch.set(speed);
      } else {
        winch.stopMotor();
        System.out.println("CLIMB GOING TOO FAR UP, STOPPING TO PREVENT CRASH " + pos + " " + speed);
      }
    } else if (belowMinimum(pos)) {
      if (speed > 0) {
        winch.set(speed);
      } else {
        winch.stopMotor();
        System.out.println("CLIMB GOING TOO FAR LOW, STOPPING TO PREVENT CRASH " + pos + " " + speed);
      }
    } else {
      winch.set(speed);
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
  public double getClimbPosition() { // get climb position
    return toInches(winch.getSelectedSensorPosition());
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
    winch.setSelectedSensorPosition(0);
  }

  /**
   * @return the raw encoder value.
   */
  public int getEncoder() { // get encoder
    return winch.getSelectedSensorPosition();
  }
  
  /**
   * Shuts off the climb chain motor.
   */
  public void stopClimb() {
    winch.stopMotor();
  }

  /**
	 * Secures the hatch in place by opening the scissors.
	 */
	public void lockClimb() {
		climbLock.set(Value.kForward);
	}

	/**
	 * Releases the hatch by closing the scissors; it will hang loosely and can fall off.
	 */
	public void loosenClimb() {
		climbLock.set(Value.kReverse);
	}

  public Value getPneumaticState() {
    return climbLock.get();
  }

  /**
	 * Shuts off power to the pickup solenoid.
	 * Use after extending or retracting; this will not move the piston.
	 */
	public void setPneumaticOff() {
		climbLock.set(Value.kOff);
  }

  // I don't believe I need a method to turn it on so it will be left out
  // I don't know if we actually need to set the pneumatic off either
  
  /**
   * Puts information about this subsystem on the dashboard.
   */
  public void putDashboard() {
    SmartDashboard.putNumber("Climb/Encoder", getEncoder());
    SmartDashboard.putNumber("Climb/Position", getClimbPosition());
    SmartDashboard.putNumber("Climb/Pneumatic State", getClimbPosition());

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (IO.verbose) putDashboard();

    
  }
  // Thank you Alex for the the Arm and Climb code
}
  