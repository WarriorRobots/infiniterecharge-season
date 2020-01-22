/*----------------------------------------------------------------------------------*/
/* MIT License                                                                      */
/*                                                                                  */
/* Copyright (c) 2019 Team 254                                                      */
/*                                                                                  */
/* Permission is hereby granted, free of charge, to any person obtaining a copy     */
/* of this software and associated documentation files (the "Software"), to deal    */
/* in the Software without restriction, including without limitation the rights     */
/* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell        */
/* copies of the Software, and to permit persons to whom the Software is            */
/* furnished to do so, subject to the following conditions:                         */
/*                                                                                  */
/* The above copyright notice and this permission notice shall be included in all   */
/* copies or substantial portions of the Software.                                  */
/*                                                                                  */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR       */
/* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,         */
/* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE      */
/* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER           */
/* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,    */
/* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE    */
/* SOFTWARE.                                                                        */
/*----------------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.control.Path;
import frc.robot.commands.autonomous.paths.PathContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivePath extends CommandBase {

  private DrivetrainSubsystem m_drive;
  private PathContainer m_PathContainer;
  private Path m_Path;
  private boolean m_StopWhenDone;

  public DrivePath(DrivetrainSubsystem drive, PathContainer p, boolean stopWhenDone) {
    m_drive = drive;
    addRequirements(m_drive);
    m_PathContainer = p;
    m_Path = m_PathContainer.buildPath();
    m_StopWhenDone = stopWhenDone;
  }

  /**
   * Follows a path and stops at the end.
   * @param p The path to be driven.
   */
  public DrivePath(DrivetrainSubsystem drive, PathContainer p) {
    this(drive, p, true);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_drive.setWantDrivePath(m_Path, m_PathContainer.isReversed());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return m_drive.isDoneWithPath();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_drive.stopDrive();
    m_drive.setOpen();
  }
}
