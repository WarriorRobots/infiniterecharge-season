package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Input Output.
 * <p>
 * All input objects (Joysticks and Xbox controllers NOT robot input), their buttons, and getters exist here.
 * <p>
 * All output settings exists here, the level of verbosity of the robot.
 */
public class IO {
  
  public static boolean verbose = true; // true if the robot will give TONS of shuffelboard information

  private static final Joystick m_leftJoystick = new Joystick(1);
  private static final Joystick m_rightJoystick = new Joystick(0);
  private static final XboxController m_xbox = new XboxController(2);

  public static double getLeftY() {
    return m_leftJoystick.getY() * -1; // * -1 because up is -1 on the joystick
  }
  
  public static double getRightY() {
    return m_rightJoystick.getY() * -1; // * -1 because up is -1 on the joystick
  }

  public static double getXBoxRightX()
  {
    return m_xbox.getX(Hand.kRight);

  }
  
}
