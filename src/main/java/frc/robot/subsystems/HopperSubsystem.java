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

public class HopperSubsystem extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */
  private WPI_TalonSRX wallSpider;
  private WPI_TalonSRX floorSpider;
  private static final int ID_WALL = 0; // TODO change value later or something
  private static final int ID_FLOOR = 0; // TODO change value later or something


  public HopperSubsystem() {
    wallSpider = new WPI_TalonSRX(ID_WALL);
    floorSpider = new WPI_TalonSRX(ID_FLOOR);


  }    
  
  public void setWallPower(double voltage)
  {
    wallSpider.set(ControlMode.PercentOutput, voltage);
  }

  public void setFloorPower(double voltage)
  {
    floorSpider.set(ControlMode.PercentOutput, voltage);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
