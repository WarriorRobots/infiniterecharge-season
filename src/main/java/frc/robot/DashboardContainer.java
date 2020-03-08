/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * A singleton tool to interface to the dashboard.
 */
public class DashboardContainer {

  private static DashboardContainer instance = null;

  private SendableChooser<Integer> verbosityChooser = new SendableChooser<Integer>();

  // This constructor is private because it is a singleton
  private DashboardContainer() {}

  private void init() {
    getTab(TabsIndex.kDriver);
    getTab(TabsIndex.kAuto);
    getTab(TabsIndex.kConfig);
  }

  /**
   * Gets the DashboardContainer instance.
   */
  public static DashboardContainer getInstance() {
    if (instance==null) {
      instance = new DashboardContainer();
      instance.init();
    }
    return instance;
  }

  /**
   * An enum to reference a tab on the dashboard.
   */
  public enum TabsIndex {
    kSmartDashboard(0,"SmartDashboard"),
    kLiveWindow(1,"LiveWindow"),
    kDriver(2,"Driver"),
    kAuto(3,"Auto"),
    kConfig(4,"Config");
    public final int index;
    public final String name;
    TabsIndex(int index, String name) {
      this.index = index;
      this.name = name;
    }
    public int getIndex(){return index;}
    public String getName(){return name;}
  }

  /**
   * Set the shuffleboard to be looking at a specific tab.
   * @param index Desired tab.
   * @return The selected tab.
   * @see TabsIndex
   */
  public ShuffleboardTab setTab(TabsIndex tabindex) {
    Shuffleboard.selectTab(tabindex.getName());
    return Shuffleboard.getTab(tabindex.getName());
  }
  
  /**
   * Get a tab from the shuffleboard.
   * @param tabindex Desired tab.
   * @return The desired tab.
   * @see TabsIndex
   */
  public ShuffleboardTab getTab(TabsIndex tabindex) {
    return Shuffleboard.getTab(tabindex.getName());
  }

  /**
   * This should ONLY be called ONCE.
   * This should also be called AFTER the RobotContainer has created all of it's subsystems.
   * This is because this will make AutoContainer put items on the dashboard and make commands that involve
   * subsystems from the RobotContainer and if the robot container did not make the items then there will be
   * issues with null items.
   */
  public void boot() {
    setupDriver();
    AutoContainer.getInstance(); // this is to get the auto to do it's tab
    // config is handled in other code
  }

  private void setupDriver() {
    ShuffleboardTab driver = getTab(TabsIndex.kDriver);
    // this is a troubleshooting note: if the limelight stream does not appear right on the dashboard:
    // did you flash it? if so, did you remember to change the team number and static ip? if no, do that
    HttpCamera limelight = new HttpCamera("limelight", "http://10.24.78.11:5800");
    driver.add("Limelight", limelight).withWidget(BuiltInWidgets.kCameraStream).withPosition(2, 0).withSize(5, 3);

    verbosityChooser.addOption("Silent", 0);
    verbosityChooser.setDefaultOption("Driver", 1);
    verbosityChooser.addOption("Driver Debug", 2);
    verbosityChooser.addOption("Middle", 3);
    verbosityChooser.addOption("Programmer", 4);
    verbosityChooser.addOption("Programmer Debug", 5);
    driver.add("Verbosity",verbosityChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 3).withSize(2, 1);
  }

  /**
   * Gets the verbosity level of the robot: <p>
   * 1 - Driver Info <p>
   * 2 - Driver Debug <p>
   * 3 - Middle (between 2 and 4) <p>
   * 4 - Programmer Info <p>
   * 5 - Programmer Debug <p>
   * 0 - Silent
   */
  public int getVerbosity() {
    // if there is a null value, then choose verbosity of 1 so something does not crash elsewhere
    return (verbosityChooser.getSelected() != null) ? verbosityChooser.getSelected() : 1;
  }
}
