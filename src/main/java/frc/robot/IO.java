/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Add your docs here.
 */
public class IO {
  
  public static boolean verbose = true; // true if the robot will give TONS of shuffelboard information

  private static final Joystick m_leftJoystick = new Joystick(1);
  private static final Joystick m_rightJoystick = new Joystick(0);

  static final JoystickButton m_right8 = new JoystickButton(m_rightJoystick,8);
  static final JoystickButton m_right10 = new JoystickButton(m_rightJoystick,10);

  public static double getLeftY() {
    return m_leftJoystick.getY() * -1; // * -1 because up is -1 on the joystick
  }
  
  public static double getRightY() {
    return m_rightJoystick.getY() * -1; // * -1 because up is -1 on the joystick
  }
  
}
