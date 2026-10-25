package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** A command that controls the intake with an Xbox controller. */
public class IntakeXboxCommand extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final CommandXboxController m_controller;

  public IntakeXboxCommand(IntakeSubsystem intakeSubsystem, CommandXboxController controller) {
    m_intakeSubsystem = intakeSubsystem;
    m_controller = controller;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Right trigger = intake in, Left trigger = intake out
    double rightTrigger = m_controller.getRightTriggerAxis();
    double leftTrigger = m_controller.getLeftTriggerAxis();

    double speed = rightTrigger - leftTrigger;
    m_intakeSubsystem.intake(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}