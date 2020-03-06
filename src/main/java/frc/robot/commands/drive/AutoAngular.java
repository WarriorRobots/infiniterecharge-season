/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * When run, the robot will turn to the provided angle,
 * using a PID loop to maintain accuracy and control.
 */
public class AutoAngular extends CommandBase {

  private DrivetrainSubsystem m_drive;

  private double m_setpoint; // degrees

  private PIDController m_PID;

  /**
   * Create a new instance of {@link TurnAuto}.
   * @param angle  What angle in degrees to turn towards. (Clockwise is postive)
   */
  public AutoAngular(DrivetrainSubsystem drive, double setpoint) {
    m_drive = drive;
    addRequirements(m_drive);

    m_setpoint = setpoint;
    
    m_PID = new PIDController(
      Vars.AUTO_ANGULAR_P,
      Vars.AUTO_ANGULAR_I,
      Vars.AUTO_ANGULAR_D);
    m_PID.setTolerance(Vars.AUTO_ANGULAR_TOLERANCE);
  }

  /**
   * Set the internal PID constants to new values
   * @param p  P gain
   * @param i  I gain
   * @param d  D gain
   */
  public void setPID(double p, double i, double d) {
    m_PID.setPID(p, i, d);
  }

  /**
   * Set an internal distance PID tolerance to a new value
   * @param tolerance Tolerance away from setpoint to be considered in range (in degrees) 
   */
  public void setTolerance(double tolerance) {
    m_PID.setTolerance(tolerance);
  }

  @Override
  public void initialize() {
    // m_drive.resetAngle();

    // try {
      // pidLoop.setIzone(-QuickAccessVars.AUTO_TURN_TOLERANCE, QuickAccessVars.AUTO_TURN_TOLERANCE);
      // pidLoop.setOutputRange(-1, 1);
    // } catch (Exception e) {}
    
    // relatively sets the setpoint
    m_PID.setSetpoint(m_drive.getAngleDegrees()+m_setpoint);
  }

  @Override
  public void execute() {
    m_drive.arcadedriveRaw(0, m_PID.calculate(m_drive.getAngleDegrees()));
  }

  @Override
  public boolean isFinished() {
    return m_PID.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    m_PID.reset();
  }
}