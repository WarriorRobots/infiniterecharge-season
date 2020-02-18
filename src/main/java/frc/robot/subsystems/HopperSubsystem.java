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

public class HopperSubsystem extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */
  private WPI_TalonSRX wallHopper;
  private WPI_TalonSRX floorHopper;


  public HopperSubsystem() {
    wallHopper = new WPI_TalonSRX(RobotMap.ID_HOPPER_A);
    floorHopper = new WPI_TalonSRX(RobotMap.ID_HOPPER_B);
  }    
  
  public void setWallPower(double voltage)
  {
    wallHopper.set(ControlMode.PercentOutput, voltage);
  }

  public void setFloorPower(double voltage)
  {
    floorHopper.set(ControlMode.PercentOutput, voltage);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
