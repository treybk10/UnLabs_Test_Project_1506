package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveXboxCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeXboxCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // The intake subsystem controlled by a TalonFX motor
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(IntakeConstants.kIntakeMotorCanId);

  // Drivetrain subsystem with MapleSim MK5n swerve simulation
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  // Shooter subsystem with three Kraken X60 motors
  private final ShooterSubsystem m_shooterSubsystem =
      new ShooterSubsystem(
          ShooterConstants.kShooterMotor1CanId,
          ShooterConstants.kShooterMotor2CanId,
          ShooterConstants.kShooterMotor3CanId);

  // Driver controller on port 0
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the intake subsystem for simulation publishing
    m_intakeSubsystem.configureSim();
    m_intakeSubsystem.configureShuffleboard();

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Returns the intake subsystem for use in simulation.
   * @return the intake subsystem
   */
  public IntakeSubsystem getIntakeSubsystem() {
    return m_intakeSubsystem;
  }

  /**
   * Returns the drivetrain subsystem for use in simulation.
   * @return the drivetrain subsystem
   */
  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return m_drivetrainSubsystem;
  }

  /**
   * Returns the shooter subsystem for use in simulation.
   * @return the shooter subsystem
   */
  public ShooterSubsystem getShooterSubsystem() {
    return m_shooterSubsystem;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // Xbox controller intake control: right trigger intakes in, left trigger intake out.
    // Runs continuously while the driver is in teleop.
    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.1
        || m_driverController.getLeftTriggerAxis() > 0.1)
        .whileTrue(new IntakeXboxCommand(m_intakeSubsystem, m_driverController));

    // Xbox controller drive control: left stick drives forward/back, right stick rotates.
    // Runs continuously while the driver is in teleop.
    new Trigger(() -> true)
        .whileTrue(new DriveXboxCommand(m_drivetrainSubsystem, m_driverController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}