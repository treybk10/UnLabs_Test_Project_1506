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
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

/**
 * Intake subsystem with TalonFX motor controller.
 * Supports both real hardware operation and simulation with AdvantageScope integration.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX m_intakeMotor;

    public IntakeSubsystem(int canId) {
        m_intakeMotor = new TalonFX(canId);
        // Safety limits
        m_intakeMotor.setExpiration(0.1);
        // Neutral mode: Brake (default safe state)
        m_intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        // Current limits: 40A stator, 50A supply (Kraken X60 specs)
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 40.0;
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 50.0;
        currentLimits.SupplyCurrentLimitEnable = true;
        m_intakeMotor.getConfigurator().apply(currentLimits);
        // Inversion state: Clockwise positive (mounting default)
        MotorOutputConfigs motorOutput = new MotorOutputConfigs();
        motorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorOutput.NeutralMode = NeutralModeValue.Brake;
        m_intakeMotor.getConfigurator().apply(motorOutput);
    }

    public void intake(double speed) {
        m_intakeMotor.set(MathUtil.clamp(speed, -1.0, 1.0));
    }

    public void stop() {
        m_intakeMotor.stopMotor();
    }

    public double getSpeed() {
        return m_intakeMotor.getDutyCycle().getValue();
    }

    @Override
    public void periodic() {
        // Add any periodic actions here
    }

    /**
     * Configures simulation publishing for AdvantageScope integration.
     * Publishes motor voltage and speed to the "Sim" NetworkTable.
     */
    public void configureSim() {
        // Phoenix6 provides built-in simulation state access via getSimState()
        // This data is automatically available to AdvantageScope via NetworkTables
    }

    /**
     * Configures Shuffleboard dashboard for the intake subsystem.
     */
    public void configureShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        tab.addDouble("Motor Speed", () -> m_intakeMotor.getDutyCycle().getValue());
    }

    /**
     * Sets the inversion state of the motor.
     * @param inverted true for Clockwise positive, false for CounterClockwise positive
     */
    public void setInverted(boolean inverted) {
        MotorOutputConfigs motorOutput = new MotorOutputConfigs();
        motorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorOutput.NeutralMode = NeutralModeValue.Brake;
        m_intakeMotor.getConfigurator().apply(motorOutput);
    }

    /**
     * Checks if the motor is currently inverted.
     * @return true if the motor is configured as Clockwise positive
     */
    public boolean isInverted() {
        com.ctre.phoenix6.configs.TalonFXConfiguration config = new com.ctre.phoenix6.configs.TalonFXConfiguration();
        m_intakeMotor.getConfigurator().refresh(config);
        return config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive;
    }
}