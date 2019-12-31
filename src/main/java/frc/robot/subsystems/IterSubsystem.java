package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * IterSubsystem is an abstract class with the structure to be looped.
 * 
 * @see SubsystemManager
 */
public abstract class IterSubsystem extends Subsystem {

  /**
   * Adds this subsystem to the subsystem manager's list.
   * 
   * @see SubsystemManager#add(IterSubsystem)
   */
  public IterSubsystem() {
    SubsystemManager.add(this);
  }

  /**
   * Command called at the start of the auto period.
   * 
   * @param t current time from the FPGA timestamp.
   */
  public abstract void onStart(double t);

  /**
   * Command called on the periodic of the auto/teleop period.
   * 
   * @param t current time from the FPGA timestamp.
   */
  public abstract void onLoop(double t);

  /**
   * Command called at the end of the teleop period.
   * 
   * @param t current time from the FPGA timestamp.
   */
  public abstract void onEnd(double t);


  /**
   * Command called periodicly (in any mode).
   * 
   * @param t current time from the FPGA timestamp.
   */
  public abstract void periodic(double t);


  /**
   * Command called periodicly when the robot is disabled.
   * 
   * @param t current time from the FPGA timestamp.
   */
  public abstract void disabled(double t);
}
