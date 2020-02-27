/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Vars;

public class IntakeSubsystem extends SubsystemBase {

  private WPI_TalonSRX m_intake;

  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {
    m_intake = new WPI_TalonSRX(RobotMap.ID_INTAKE);
    m_intake.setInverted(Vars.INTAKE_REVERSED);
  }

  public void intakeAtPercent(double percent) {
    m_intake.set(ControlMode.PercentOutput, percent);
  }

  public void stop() {
    m_intake.stopMotor();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
