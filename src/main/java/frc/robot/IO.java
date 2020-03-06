package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.triggers.DpadTrigger;
import frc.robot.util.triggers.ThresholdTrigger;

/**
 * Input Output.
 * <p>
 * All input objects (Joysticks and Xbox controllers NOT robot input), their buttons, and getters exist here.
 * <p>
 * All output settings exists here, the level of verbosity of the robot.
 */
public class IO {

  /**
   * 5 is the most information
   * 4 - 2 is middle information
   * 1 is driver information
   * 0 is silent
   */
  public static int verbose = 5; // true if the robot will give TONS of shuffleboard information TODO make the verbosity be determined by a dashboard value

  private static final Joystick m_leftJoystick = new Joystick(1);
  private static final Joystick m_rightJoystick = new Joystick(0);
  private static final XboxController m_xbox = new XboxController(2);

  protected static final JoystickButton leftJoystick_1 = new JoystickButton(m_leftJoystick, 1);
  protected static final JoystickButton leftJoystick_2 = new JoystickButton(m_leftJoystick, 2);
  protected static final JoystickButton leftJoystick_3 = new JoystickButton(m_leftJoystick, 3);
  protected static final JoystickButton leftJoystick_4 = new JoystickButton(m_leftJoystick, 4);
  protected static final JoystickButton leftJoystick_5 = new JoystickButton(m_leftJoystick, 5);
  protected static final JoystickButton leftJoystick_6 = new JoystickButton(m_leftJoystick, 6);
  protected static final JoystickButton leftJoystick_7 = new JoystickButton(m_leftJoystick, 7);
  protected static final JoystickButton leftJoystick_8 = new JoystickButton(m_leftJoystick, 8);
  protected static final JoystickButton leftJoystick_9 = new JoystickButton(m_leftJoystick, 9);
  protected static final JoystickButton leftJoystick_10 = new JoystickButton(m_leftJoystick, 10);
  protected static final JoystickButton leftJoystick_11 = new JoystickButton(m_leftJoystick, 11);
  protected static final JoystickButton leftJoystick_12 = new JoystickButton(m_leftJoystick, 12);

  protected static final JoystickButton rightJoystick_1 = new JoystickButton(m_rightJoystick, 1);
  protected static final JoystickButton rightJoystick_2 = new JoystickButton(m_rightJoystick, 2);
  protected static final JoystickButton rightJoystick_3 = new JoystickButton(m_rightJoystick, 3);
  protected static final JoystickButton rightJoystick_4 = new JoystickButton(m_rightJoystick, 4);
  protected static final JoystickButton rightJoystick_5 = new JoystickButton(m_rightJoystick, 5);
  protected static final JoystickButton rightJoystick_6 = new JoystickButton(m_rightJoystick, 6);
  protected static final JoystickButton rightJoystick_7 = new JoystickButton(m_rightJoystick, 7);
  protected static final JoystickButton rightJoystick_8 = new JoystickButton(m_rightJoystick, 8);
  protected static final JoystickButton rightJoystick_9 = new JoystickButton(m_rightJoystick, 9);
  protected static final JoystickButton rightJoystick_10 = new JoystickButton(m_rightJoystick, 10);
  protected static final JoystickButton rightJoystick_11 = new JoystickButton(m_rightJoystick, 11);
  protected static final JoystickButton rightJoystick_12 = new JoystickButton(m_rightJoystick, 12);

  protected static final JoystickButton xbox_A = new JoystickButton(m_xbox, 1);
  protected static final JoystickButton xbox_B= new JoystickButton(m_xbox, 2);
  protected static final JoystickButton xbox_X = new JoystickButton(m_xbox, 3);
  protected static final JoystickButton xbox_Y = new JoystickButton(m_xbox, 4);
  protected static final JoystickButton xbox_LB = new JoystickButton(m_xbox, 5);
  protected static final JoystickButton xbox_RB = new JoystickButton(m_xbox, 6);
  protected static final JoystickButton xbox_SELECT = new JoystickButton(m_xbox, 7);
  protected static final JoystickButton xbox_START = new JoystickButton(m_xbox, 8);
  protected static final JoystickButton xbox_L_JOYSTICK = new JoystickButton(m_xbox, 9);
  protected static final JoystickButton xbox_R_JOYSTICK = new JoystickButton(m_xbox, 10);

  protected static final DpadTrigger xboxUp = new DpadTrigger(() -> m_xbox.getPOV(), 0);
  protected static final DpadTrigger xboxDown = new DpadTrigger(() -> m_xbox.getPOV(), 180);
  protected static final DpadTrigger xboxLeft = new DpadTrigger(() -> m_xbox.getPOV(), 270);
  protected static final DpadTrigger xboxRight = new DpadTrigger(() -> m_xbox.getPOV(), 90);

  protected static final ThresholdTrigger xbox_LT = new ThresholdTrigger(() -> m_xbox.getTriggerAxis(Hand.kLeft), 0.9);
  protected static final ThresholdTrigger xbox_RT = new ThresholdTrigger(() -> m_xbox.getTriggerAxis(Hand.kRight), 0.9);
  protected static final ThresholdTrigger xbox_R_UP = new ThresholdTrigger(() -> getXBoxRightY(), 0.9);
  protected static final ThresholdTrigger xbox_R_DOWN = new ThresholdTrigger(() -> getXBoxRightY(), -0.9);

  
  protected static double getLeftX() {
    return m_leftJoystick.getX();
  }
  
  protected static double getLeftY() {
    return m_leftJoystick.getY() * -1; // * -1 because up is -1 on the joystick
  }
  
  protected static double getLeftZ() {
    return m_leftJoystick.getZ();
  }
  
  protected static double getLeftSlider()  {
    return m_leftJoystick.getRawAxis(3); //The third axis is slider
  }
  
  protected static double getRightX() {
    return m_rightJoystick.getX();
  }
  
  protected static double getRightY() {
    return m_rightJoystick.getY() * -1; // * -1 because up is -1 on the joystick
  }
  
  protected static double getRightZ() {
    return m_rightJoystick.getZ();
  }

  protected static double getRightSlider()  {
    return m_rightJoystick.getRawAxis(3); //The third axis is the slider
  } 
  
  protected static double getXBoxLeftX()  {
    return m_xbox.getX(Hand.kLeft);
  }

  protected static double getXBoxLeftY()
  {
    return m_xbox.getY(Hand.kLeft) * -1; // * -1 because up is -1 on the joystick
  }

  protected static double getXBoxRightX() {
    return m_xbox.getX(Hand.kRight);
  }

  protected static double getXBoxRightY() {
    return m_xbox.getY(Hand.kRight) * -1; // * -1 because up is -1 on the joystick
  }
  
  protected static double getXBoxLeftTrigger()  {
    return m_xbox.getTriggerAxis(Hand.kLeft);
  }
  
  protected static double getXBoxRightTrigger() {
    return m_xbox.getTriggerAxis(Hand.kRight);
  }

}
