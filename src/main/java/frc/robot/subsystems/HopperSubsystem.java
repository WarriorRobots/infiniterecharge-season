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

public class HopperSubsystem extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */
  private WPI_TalonSRX m_wall;
  private WPI_TalonSRX m_floor;


  public HopperSubsystem() {
    m_wall = new WPI_TalonSRX(RobotMap.ID_HOPPER_WALL);
    m_wall.setInverted(Vars.HOPPER_WALL_REVERSED);
    m_floor = new WPI_TalonSRX(RobotMap.ID_HOPPER_FLOOR);
    m_floor.setInverted(Vars.HOPPER_FLOOR_REVERSED);
  }    
  
  public void setWallPower(double percent)
  {
    m_wall.set(ControlMode.PercentOutput, percent);
  }

  public void setFloorPower(double percent)
  {
    m_floor.set(ControlMode.PercentOutput, percent);
  }

  public void stop() {
    m_wall.stopMotor();
    m_floor.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
