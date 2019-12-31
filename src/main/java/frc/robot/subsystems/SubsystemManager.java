package frc.robot.subsystems;

import java.util.ArrayList;

/**
 * A class to keep an arrylist of all itersubsystems that need to be iterated and iterate them when this manager is called.
 * 
 * @see IterSubsystem
 */
public class SubsystemManager {
  /*
   * NOTE: The Subsystem Manager can NOT extend IterSubsystem because then it would add itself to the list of things to be iterated.
   *           Then when any command is called in SubsystemManager, it would eventually call itself (as it was added to the list) and iterate itself... forever.
   */

  private static ArrayList<IterSubsystem> subsystems = new ArrayList<IterSubsystem>();

  private SubsystemManager() {}

  /**
   * Adds an itersubsystem to the manager's list of subsystems.
   */
  protected static void add(IterSubsystem subsystem) {
    subsystems.add(subsystem);
  }

  /**
   * Calls all itersubsystem's onStart command.
   * 
   * @param t current time from the FPGA timestamp.
   * @see IterSubsystem
   */
  public static void onStart(double t) {
    for (IterSubsystem s : subsystems) {
      s.onStart(t);
    }
  }

  /**
   * Calls all itersubsystem's onLoop command.
   * 
   * @param t current time from the FPGA timestamp.
   * @see IterSubsystem
   */
  public static void onLoop(double t) {
    for (IterSubsystem s : subsystems) {
      s.onLoop(t);
    }
  }

  /**
   * Calls all itersubsystem's onLoop command.
   * 
   * @param t current time from the FPGA timestamp.
   * @see IterSubsystem
   */
  public static void onEnd(double t) {
    for (IterSubsystem s : subsystems) {
      s.onEnd(t);
    }
  }

  /**
   * Calls all itersubsystem's periodic command.
   * 
   * @param t current time from the FPGA timestamp.
   * @see IterSubsystem
   */
  public static void periodic(double t) {
    for (IterSubsystem s : subsystems) {
      s.periodic(t);
    }
  }

  /**
   * Calls all itersubsystem's disabled command.
   * 
   * @param t current time from the FPGA timestamp.
   * @see IterSubsystem
   */
  public static void disabled(double t) {
    for (IterSubsystem s : subsystems) {
      s.disabled(t);
    }
  }

}
