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
    double angle = (double) m_pixy.getAngleX();
    double ballOffset = m_pixy.getNecessaryOffset();
    double angleDifference = angle - ballOffset;
    if(angleDifference > 5) {
      m_drive.arcadedriveRaw(0,-0.3);
    }
    if(angleDifference < 5) {
      m_drive.arcadedriveRaw(0,0.3);
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
