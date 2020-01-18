/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.
 */
public class IO {

  private static final Joystick m_leftJoystick = new Joystick(1);
  private static final Joystick m_rightJoystick = new Joystick(0);

  public static double getLeftY() {
    return m_leftJoystick.getY() * -1; // * -1 because up is -1 on the joystick
  }
  
  public static double getRightY() {
    return m_rightJoystick.getY() * -1; // * -1 because up is -1 on the joystick
  }
  
}
