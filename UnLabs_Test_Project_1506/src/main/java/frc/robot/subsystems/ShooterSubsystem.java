// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Shooter subsystem with three Kraken X60 motor controllers.
 * Runs all three motors at set velocities in rotations per second.
 */
public class ShooterSubsystem extends SubsystemBase {

  private static final double kMaxRotationsPerSecond = 20.0;

  private final TalonFX m_shooterMotor1;
  private final TalonFX m_shooterMotor2;
  private final TalonFX m_shooterMotor3;

  private final DoublePublisher m_motor1SpeedPublisher;
  private final DoublePublisher m_motor2SpeedPublisher;
  private final DoublePublisher m_motor3SpeedPublisher;

  /**
   * Creates a new ShooterSubsystem with three Kraken X60 motor controllers.
   *
   * @param canId1 CAN ID for the first Kraken X60 motor
   * @param canId2 CAN ID for the second Kraken X60 motor
   * @param canId3 CAN ID for the third Kraken X60 motor
   */
  public ShooterSubsystem(int canId1, int canId2, int canId3) {
    m_shooterMotor1 = new TalonFX(canId1);
    m_shooterMotor2 = new TalonFX(canId2);
    m_shooterMotor3 = new TalonFX(canId3);

    // Configure all three motors with safety limits, current limits, and inversion states
    configureMotor(m_shooterMotor1);
    configureMotor(m_shooterMotor2);
    configureMotor(m_shooterMotor3);

    // NetworkTable for AdvantageScope simulation data publishing
    NetworkTable simTable = NetworkTableInstance.getDefault().getTable("Sim");

    m_motor1SpeedPublisher = simTable.getDoubleTopic("Shooter.Motor1Speed").publish();
    m_motor2SpeedPublisher = simTable.getDoubleTopic("Shooter.Motor2Speed").publish();
    m_motor3SpeedPublisher = simTable.getDoubleTopic("Shooter.Motor3Speed").publish();
  }

  /**
   * Configures a single motor with safety limits, current limits, and default inversion.
   * Kraken X60 typical specs: 60A stator current limit, 80A supply current limit.
   */
  private void configureMotor(TalonFX motor) {
    // Safety expiration: 0.1 seconds
    motor.setExpiration(0.1);

    // Neutral mode: Brake (default safe state)
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.NeutralMode = NeutralModeValue.Brake;
    motorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor.getConfigurator().apply(motorOutput);

    // Current limits: 60A stator, 80A supply (Kraken X60 continuous specs)
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit = 60.0;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 80.0;
    currentLimits.SupplyCurrentLimitEnable = true;
    motor.getConfigurator().apply(currentLimits);
  }

  /**
   * Sets all three shooter motors to the specified velocity in rotations per second.
   * The speed is clamped to a reasonable range for the Kraken X60.
   *
   * @param rotationsPerSecond the desired velocity in rotations per second (RPS)
   *                         Typical range: -20.0 to 20.0 RPS for Kraken X60
   */
  public void setRotationsPerSecond(double rotationsPerSecond) {
    // Clamp to reasonable Kraken X60 velocity range
    double clampedSpeed = MathUtil.clamp(rotationsPerSecond, -kMaxRotationsPerSecond, kMaxRotationsPerSecond);

    m_shooterMotor1.set(clampedSpeed);
    m_shooterMotor2.set(clampedSpeed);
    m_shooterMotor3.set(clampedSpeed);

    // Publish simulated speed to NetworkTable
    m_motor1SpeedPublisher.set(clampedSpeed);
    m_motor2SpeedPublisher.set(clampedSpeed);
    m_motor3SpeedPublisher.set(clampedSpeed);
  }

  /**
   * Sets the inversion state for all three motors.
   * Inverting a motor reverses its direction.
   *
   * @param inverted true to invert all motors, false for normal direction
   */
  public void setInverted(boolean inverted) {
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    motorOutput.NeutralMode = NeutralModeValue.Brake;

    m_shooterMotor1.getConfigurator().apply(motorOutput);
    m_shooterMotor2.getConfigurator().apply(motorOutput);
    m_shooterMotor3.getConfigurator().apply(motorOutput);
  }

  /**
   * Stops all three shooter motors.
   */
  public void stop() {
    m_shooterMotor1.stopMotor();
    m_shooterMotor2.stopMotor();
    m_shooterMotor3.stopMotor();
  }

  /**
   * Returns the current duty cycle (output %) of the first shooter motor.
   *
   * @return the current motor output as a duty cycle (-1.0 to 1.0)
   */
  public double getMotor1Output() {
    return m_shooterMotor1.getDutyCycle().getValue();
  }

  /**
   * Returns the current duty cycle (output %) of the second shooter motor.
   *
   * @return the current motor output as a duty cycle (-1.0 to 1.0)
   */
  public double getMotor2Output() {
    return m_shooterMotor2.getDutyCycle().getValue();
  }

  /**
   * Returns the current duty cycle (output %) of the third shooter motor.
   *
   * @return the current motor output as a duty cycle (-1.0 to 1.0)
   */
  public double getMotor3Output() {
    return m_shooterMotor3.getDutyCycle().getValue();
  }

  /**
   * Checks if the motors are currently inverted.
   *
   * @return true if the motors are configured as CounterClockwise positive
   */
  public boolean isInverted() {
    com.ctre.phoenix6.configs.TalonFXConfiguration config = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    m_shooterMotor1.getConfigurator().refresh(config);
    return config.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive;
  }

  @Override
  public void periodic() {
    // Add any periodic actions here
  }

  /**
   * Configures simulation publishing for AdvantageScope integration.
   * Publishes motor speeds to the "Sim" NetworkTable.
   */
  public void configureSim() {
    // Motor speeds are already published via the publishers in the constructor
  }

  /**
   * Configures Shuffleboard dashboard for the shooter subsystem.
   */
  public void configureShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    tab.addDouble("Motor 1 Speed", () -> m_shooterMotor1.getDutyCycle().getValue());
    tab.addDouble("Motor 2 Speed", () -> m_shooterMotor2.getDutyCycle().getValue());
    tab.addDouble("Motor 3 Speed", () -> m_shooterMotor3.getDutyCycle().getValue());
  }
}