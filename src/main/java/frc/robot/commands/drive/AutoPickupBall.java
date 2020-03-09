/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PixyCamSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Vars;

public class AutoPickupBall extends CommandBase {
  PixyCamSubsystem m_pixy;
  DrivetrainSubsystem m_drive;
  IntakeSubsystem m_intake;

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
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
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

    if(robot_angle > 5) {
      m_drive.arcadedriveRaw(0, -.25);
    }
    if(robot_angle < -5) {
      m_drive.arcadedriveRaw(0, .25);
    }
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
