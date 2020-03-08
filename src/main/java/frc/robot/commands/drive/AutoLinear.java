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
 * When run, the robot will drive straight at the provided distance,
 * using a PID loop to stay on-course.
 */
public class AutoLinear extends CommandBase {

  DrivetrainSubsystem m_drive;

  private double m_setpoint; // inches

  private PIDController pidAngle, pidDistance;

  /**
   * Create a new instance of {@link DriveAuto}.
   * @param drive Drivetrain subsystem
   * @param m_setpoint How many inches to travel.
   */
  public AutoLinear(DrivetrainSubsystem drive, double setpoint) {
    m_drive = drive;
    addRequirements(m_drive);
    
    m_setpoint = setpoint;

    // sets default pid values
    pidDistance = new PIDController(
      Vars.AUTO_LINEAR_P,
      Vars.AUTO_LINEAR_I,
      Vars.AUTO_LINEAR_D);
    pidAngle = new PIDController(
      Vars.AUTO_LINEAR_ANGLE_P,
      0,
      0);
    pidDistance.setTolerance(Vars.AUTO_LINEAR_TOLERANCE);
  }

  /**
   * Set the internal distance PID constants to new values
   * @param p  P gain
   * @param i  I gain
   * @param d  D gain
   */
  public void setDistancePid(double p, double i, double d) {
    pidDistance.setPID(p, i, d);
  }

  /**
   * Set an internal distance PID tolerance to a new value
   * @param tolerance Tolerance away from setpoint to be considered in range (in inches) 
   */
  public void setDistanceTolerance(double tolerance) {
    pidDistance.setTolerance(tolerance);
  }

  /**
   * Set the internal angular PID constants to new values
   * @param p  P gain
   * @param i  I gain
   * @param d  D gain
   */
  public void setAngularPid(double p, double i, double d) {
    pidAngle.setPID(p, i, d);
  }
	
	@Override
	public void initialize() {
    // m_drive.reset();

    // relatively sets the setpoint
    pidDistance.setSetpoint(m_drive.getAveragePosition()+m_setpoint);
    // pidDistance.setOutputRange(-0.75, 0.75);
    // pidDistance.setIzone(-0.15, 0.15);
    pidAngle.setSetpoint(m_drive.getAngleDegrees()); // straight forwards
	}
	
	@Override
	public void execute() {
    m_drive.arcadedriveRaw(
      pidDistance.calculate(m_drive.getAveragePosition()),
      pidAngle.calculate(m_drive.getAngleDegrees())
    );
  }

	@Override
	public boolean isFinished() {
    return pidDistance.atSetpoint();
	}
	
	@Override
	public void end(boolean interrupted) {
    m_drive.stop();
    pidDistance.reset();
    pidAngle.reset(); 
  }
}