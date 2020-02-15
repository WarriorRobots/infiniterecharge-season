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
import frc.robot.Vars;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private static final int ID_SPINNY = 0; // TODO GO BACK LATER TO FIX ID

  private WPI_TalonSRX Spinny;
  public IntakeSubsystem() {
    Spinny = new WPI_TalonSRX(ID_SPINNY);
  }

  // Spins the motor at some low value and other for a specific percent, 

  public void feedAtPercent(double voltage)
  {
    Spinny.set(ControlMode.PercentOutput, voltage);
  }

  public void lowFeed()
  {
    Spinny.set(ControlMode.PercentOutput, Vars.INTAKE_LOW_FEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
