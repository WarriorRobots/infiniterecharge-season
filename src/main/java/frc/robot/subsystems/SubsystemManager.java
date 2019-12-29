package frc.robot.subsystems;

import java.util.ArrayList;

public class SubsystemManager extends IterSubsystem {
  private static SubsystemManager instance;
  private ArrayList<IterSubsystem> subsystems;

  public SubsystemManager() {
    instance = this;
    subsystems = new ArrayList<IterSubsystem>();
  }

  /**
   * Adds a subsystem to the manager's list of subsystems.
   */
  protected static void add(IterSubsystem subsystem) {
    instance.subsystems.add(subsystem);
  }

  /**
   * Calls all subsystem's onStart command.
   * 
   * @param t current time from the FPGA timestamp.
   */
  public void onStart(double t) {
    for (IterSubsystem s : subsystems) {
      s.onStart(t);
    }
  }

  /**
   * Calls all subsystem's onLoop command.
   * 
   * @param t current time from the FPGA timestamp.
   */
  public void onLoop(double t) {
    for (IterSubsystem s : subsystems) {
      s.onLoop(t);
    }
  }

  /**
   * Calls all subsystem's onLoop command.
   * 
   * @param t current time from the FPGA timestamp.
   */
  public void onEnd(double t) {
    for (IterSubsystem s : subsystems) {
      s.onEnd(t);
    }
  }

  /**
   * Calls all subsystem's periodic command.
   * 
   * @param t current time from the FPGA timestamp.
   */
  public void periodic(double t) {
    for (IterSubsystem s : subsystems) {
      s.periodic(t);
    }
  }

  /**
   * Calls all subsystem's disabled command.
   * 
   * @param t current time from the FPGA timestamp.
   */
  public void disabled(double t) {
    for (IterSubsystem s : subsystems) {
      s.disabled(t);
    }
  }


  public void initDefaultCommand() {}

}
