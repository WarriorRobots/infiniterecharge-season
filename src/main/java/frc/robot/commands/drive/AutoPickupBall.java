/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PixyCamSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Vars;

public class AutoPickupBall extends CommandBase {
  PixyCamSubsystem m_pixy;
  DrivetrainSubsystem m_drive;
  IntakeSubsystem m_intake;

  PIDController pidDistance, pidAngle;

  /**
   * Creates a new AutoPickupBall.
   */
  public AutoPickupBall(PixyCamSubsystem pixy, DrivetrainSubsystem drivetrain, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pixy = pixy;
    addRequirements(this.m_pixy);
    m_drive = drivetrain;
    addRequirements(this.m_drive);
    m_intake = intake;
    addRequirements(this.m_intake);

    pidDistance = new PIDController(
      Vars.AUTO_BALL_PICKUP_DISTANCE_P, 
      Vars.AUTO_BALL_PICKUP_DISTANCE_I, 
      Vars.AUTO_BALL_PICKUP_DISTANCE_D);
    pidAngle = new PIDController(
      Vars.AUTO_BALL_PICKUP_ANGLE_P,
      0,
      0);
    pidDistance.setTolerance(Vars.AUTO_BALL_PICKUP_TOLERANCE);
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
    pidDistance.setSetpoint(0);
    pidAngle.setSetpoint(0);
	}

  @Override
  public void execute() {
    // all calculations can be found on pg 37 of the engineering notebook
    double a;
    double b = Vars.PIXY_INTAKE_OFFSET;
    double c = m_pixy.PIXY_BALL_DISTANCE;
    double A = Vars.PIXY_FoV_OFFSET - m_pixy.PIXY_BALL_ANGLE;
    double C;
    double a_squared = b * b + c * c - (2 * b * c * Math.cos(A));
    double robot_angle;
    double robot_distance;

    a = Math.sqrt(a_squared);
    robot_distance = a;
    if(c * c > a_squared + b * b) {
      C = 180 - A - Math.asin(b * Math.sin(A) / a);
    }
    else if(c * c < a_squared + b * b) {
      C = Math.asin(c * Math.sin(A) / a);
    }
    else {
      return;
    }
    robot_angle = C - 90 - 23.5;

    m_drive.arcadedriveRaw(
      pidDistance.calculate(robot_distance),
      pidAngle.calculate(robot_angle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
