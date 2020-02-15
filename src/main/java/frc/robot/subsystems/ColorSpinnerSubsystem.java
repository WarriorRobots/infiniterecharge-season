/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;



// JOSE CODE JOSE CODE

public class ColorSpinnerSubsystem extends SubsystemBase {
  private WPI_TalonSRX spinner;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  String colorString;

  public ColorSpinnerSubsystem() {
    spinner = new WPI_TalonSRX(RobotMap.ID_DISK);
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);   
  }

  /**
   * Gets the current color of the wheel (note this is 90 degrees off of what the field reads.)
   * @return Color as a string: Blue, Red, Green, Yellow, Unknown
   */
  public String getColor() {
    Color detectedColor = m_colorSensor.getColor();
    // gets color from color sensor
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kBlueTarget) {
      colorString = "B";
    } else if (match.color == kRedTarget) {
      colorString = "R";
    } else if (match.color == kGreenTarget) {
      colorString = "G";
    } else if (match.color == kYellowTarget) {
      colorString = "Y";
    } else {
      colorString = "Unknown";
    }
    return colorString;
  }

  /**
   * Spin the wheel with a constant voltage.
   * @param voltage from -1 to 1 to spin the wheel.
   */
  public void spinWheel(double voltage) 
  {
    spinner.set(ControlMode.PercentOutput, voltage);
  }

  /** stops the motor */
  public void stop()
  {
    spinner.stopMotor();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
