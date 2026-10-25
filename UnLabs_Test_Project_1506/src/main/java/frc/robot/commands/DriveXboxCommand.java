// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** A command that controls the drivetrain with an Xbox controller. */
public class DriveXboxCommand extends Command {
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final CommandXboxController m_controller;

  public DriveXboxCommand(DrivetrainSubsystem drivetrainSubsystem, CommandXboxController controller) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_controller = controller;
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Left stick Y axis: forward/back (invert so up is forward)
    double vx = -m_controller.getLeftY();
    // Left stick X axis: strafe left/right
    double vy = m_controller.getLeftX();
    // Right stick X axis: rotation
    double omega = m_controller.getRightX();

    // Apply deadband to reduce noise
    double deadband = 0.1;
    if (Math.abs(vx) < deadband) vx = 0.0;
    if (Math.abs(vy) < deadband) vy = 0.0;
    if (Math.abs(omega) < deadband) omega = 0.0;

    // Set robot speeds in simulation using field-relative control
    m_drivetrainSubsystem.setSimulationSpeeds(
      new ChassisSpeeds(vx, vy, omega, m_drivetrainSubsystem.getSimulatedPose().getRotation())
    );
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when command ends
    m_drivetrainSubsystem.setSimulationSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
