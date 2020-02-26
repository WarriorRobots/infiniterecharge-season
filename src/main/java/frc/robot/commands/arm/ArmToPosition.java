/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToPosition extends InstantCommand {

  ArmSubsystem m_arm;
  double m_position;

  /**
   * Command the arm (only once) to go to a position
   * @param arm Arm subsystem
   * @param position desired position of arm
   */
  public ArmToPosition(ArmSubsystem arm, double position) {
    m_arm = arm;
    addRequirements(m_arm);
    m_position = position;
  }

  @Override
  public void initialize() {
    m_arm.rotateToPosition(m_position);
  }
}
