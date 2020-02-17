/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.camera;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;

public class CameraChangePipeline extends CommandBase {
  CameraSubsystem m_camera;
	int pipeline;

	/**
	 * Changes the pipeline ID of the limelight, which switches the vision tracking settings.
	 * Use the static variables in CameraSubsystem.
	 */
	public CameraChangePipeline(CameraSubsystem camera, int pipeline) {
    m_camera = camera;
    addRequirements(this.m_camera);
		this.pipeline = pipeline;
	}

	@Override
	public void initialize() {
		m_camera.setPipeline(pipeline);
		System.out.println("Camera: Running " + this.getClass().getSimpleName());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
