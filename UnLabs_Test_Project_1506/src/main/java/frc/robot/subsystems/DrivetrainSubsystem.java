package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * Drivetrain subsystem with MapleSim integration for MK5n swerve modules using Kraken X60 motors.
 * Simulates a 4-module swerve drive with a single pose (x, y, heading) and provides
 * simulation data publishing to NetworkTable for AdvantageScope integration.
 */
public class DrivetrainSubsystem extends SubsystemBase {

  private final SwerveDriveSimulation m_driveSimulation;
  private final SwerveModuleSimulation[] m_moduleSimulations;

  // Module translation positions (FL, FR, BL, BR) for a 0.711m square robot
  // Centered at origin, half-dimensions = 0.711 / 2 = 0.3555 m
  private static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
      new Translation2d(0.3555, 0.3555), // Front-Left
      new Translation2d(0.3555, -0.3555), // Front-Right
      new Translation2d(-0.3555, 0.3555), // Back-Left
      new Translation2d(-0.3555, -0.3555) // Back-Right
  };

  // NetworkTable for AdvantageScope simulation data publishing
  private final NetworkTable m_simTable;
  // Publish robot pose as a structured Pose3d entity for AdvantageScope field visualization
  // AdvantageScope can consume this as a structured pose that can be dragged to the Poses field
  private final StructPublisher<Pose3d> m_posePublisher;
  // Publish fuel game piece poses as an array of Pose3d for AdvantageScope fuel visualization
  private final StructArrayPublisher<Pose3d> m_fuelPosesPublisher;
  private final DoublePublisher m_chassisVxPublisher;
  private final DoublePublisher m_chassisVyPublisher;
  private final DoublePublisher m_chassisOmegaPublisher;

  /**
   * Creates a new DrivetrainSubsystem with MapleSim swerve drive simulation.
   * Configures 4 MK5n swerve modules with Kraken X60 motors.
   */
  public DrivetrainSubsystem() {
    // Create MK5n module configurations using Kraken X60 motors
    // Gear ratio level 2 gives a 5.9:1 reduction for MK5n
    SwerveModuleSimulationConfig moduleConfig =
        COTS.ofMark4n(
            DCMotor.getKrakenX60(1), // Drive motor: Kraken X60 (1 motor)
            DCMotor.getKrakenX60(1), // Steer motor: Kraken X60 (1 motor)
            COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof, // Wheel coefficient of friction (1.426)
            2); // Gear ratio level 2 (5.9:1)

    // Gyro simulation using Pigeon2 (as per MK5n recommendation)
    GyroSimulation gyroSimulation = COTS.ofPigeon2().get();

    // Build the drive train simulation config
    // Robot mass: 50.0 kg from RobotSimModel.yaml
    // Bumper dimensions: 0.711m x 0.711m from RobotSimModel.yaml (same as robot width/length)
    // Track dimensions: 0.546m from RobotSimModel.yaml (21.5 inches between wheels)
    DriveTrainSimulationConfig config = new DriveTrainSimulationConfig(
        Units.Kilograms.of(50.0), // robot mass from RobotSimModel.yaml
        Units.Meters.of(0.711), // bumper length X (28 inches)
        Units.Meters.of(0.711), // bumper width Y (28 inches)
        Units.Meters.of(0.546), // track length X (21.5 inches)
        Units.Meters.of(0.546), // track width Y (21.5 inches)
        () -> gyroSimulation, // gyro simulation factory
        // Module simulation factory (same config for all 4 modules: FL, FR, BL, BR)
        () -> new SwerveModuleSimulation(moduleConfig));

    // Override with custom module translations for our robot geometry
    // (moduleTranslations set by the constructor based on trackWidth/trackLength;
    //  we override them with our actual module positions for a square robot)
    config = config.withCustomModuleTranslations(MODULE_TRANSLATIONS);

    // Spawn at center of 2026 field to avoid starting in the wall
    // Field dimensions: ~16.54m x 8.02m, center is approximately (8.27, 4.01)
    m_driveSimulation =
        new SwerveDriveSimulation(config, new Pose2d(8.27, 4.01, Rotation2d.kZero));

    // Retrieve the module simulations created by SwerveDriveSimulation
    m_moduleSimulations = m_driveSimulation.getModules();

    // Initialize NetworkTable for AdvantageScope integration
    m_simTable = NetworkTableInstance.getDefault().getTable("Sim");

    // Publish robot pose as a structured Pose3d entity for AdvantageScope field visualization
    // AdvantageScope can consume this structured pose and drag it to the Poses field
    // Using Pose3d.struct (WPILib 2026.2.1 provides a static Struct field for Pose3d)
    m_posePublisher =
        m_simTable.getStructTopic("Drivetrain/Pose", Pose3d.struct).publish();

    // Publish fuel game piece poses as an array of Pose3d for AdvantageScope fuel visualization
    m_fuelPosesPublisher =
        m_simTable.getStructArrayTopic("Drivetrain/FuelPoses", Pose3d.struct).publish();

    // Publish chassis velocities for additional control
    m_chassisVxPublisher = m_simTable.getDoubleTopic("Drivetrain.Vx").publish();
    m_chassisVyPublisher = m_simTable.getDoubleTopic("Drivetrain.Vy").publish();
    m_chassisOmegaPublisher =
        m_simTable.getDoubleTopic("Drivetrain.Omega").publish();
  }

  /**
   * Steps the drivetrain simulation forward one timestep.
   * Should be called from the robot's simulationPeriodic() method.
   */
  public void simulationPeriodic() {
    // Step the swerve drive simulation
    m_driveSimulation.simulationSubTick();

    // Get the current simulated pose (single pose)
    Pose2d currentPose2d = m_driveSimulation.getSimulatedDriveTrainPose();
    // Convert Pose2d to Pose3d for AdvantageScope field visualization (structured pose)
    // Use Rotation3d(roll, pitch, yaw) where yaw is the 2D heading in radians
    Rotation3d rotation3d =
        new Rotation3d(0.0, 0.0, currentPose2d.getRotation().getRadians());
    Pose3d currentPose3d =
        new Pose3d(
            currentPose2d.getX(),
            currentPose2d.getY(),
            0.0,
            rotation3d);
    // Publish as a structured pose that AdvantageScope can consume and drag to the Poses field
    m_posePublisher.set(currentPose3d);

    // Get the current chassis speeds
    ChassisSpeeds chassisSpeeds =
        m_driveSimulation.getDriveTrainSimulatedChassisSpeedsRobotRelative();
    m_chassisVxPublisher.set(chassisSpeeds.vxMetersPerSecond);
    m_chassisVyPublisher.set(chassisSpeeds.vyMetersPerSecond);
    m_chassisOmegaPublisher.set(chassisSpeeds.omegaRadiansPerSecond);

    // Publish fuel game piece poses for AdvantageScope fuel visualization
    // Use the singleton arena instance to get all fuel poses
    org.ironmaple.simulation.SimulatedArena arena =
        org.ironmaple.simulation.SimulatedArena.getInstance();
    if (arena != null) {
      Pose3d[] fuelPoses = arena.getGamePiecesArrayByType("Fuel");
      m_fuelPosesPublisher.set(fuelPoses);
    }
  }

  /**
   * Returns the current simulated pose (x, y, heading) of the drivetrain.
   *
   * @return the current robot pose in simulation world coordinates
   */
  public Pose2d getSimulatedPose() {
    return m_driveSimulation.getSimulatedDriveTrainPose();
  }

  /**
   * Returns the current chassis speeds from the simulation.
   *
   * @return the current chassis speeds, robot-relative
   */
  public ChassisSpeeds getSimulatedChassisSpeeds() {
    return m_driveSimulation.getDriveTrainSimulatedChassisSpeedsRobotRelative();
  }

  /**
   * Returns the current state of all swerve modules.
   *
   * @return array of SwerveModuleState for each module (FL, FR, BL, BR)
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = m_moduleSimulations[i].getCurrentState();
    }
    return states;
  }

  /**
   * Returns the current module positions (encoder positions) for odometry.
   *
   * @return array of SwerveModulePosition for each module (FL, FR, BL, BR)
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      double wheelRotationMeters =
          m_moduleSimulations[i]
                  .getDriveWheelFinalPosition()
                  .times(m_moduleSimulations[i].config.WHEEL_RADIUS.in(Units.Meters))
              .in(Units.Radians);
      positions[i] =
          new SwerveModulePosition(
              wheelRotationMeters,
              m_moduleSimulations[i].getSteerAbsoluteFacing());
    }
    return positions;
  }

  /**
   * Returns the kinematics object for this drivetrain.
   * Used for converting between chassis speeds and module states.
   *
   * @return the SwerveDriveKinematics object
   */
  public SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(MODULE_TRANSLATIONS);
  }

  /**
   * Sets the simulated robot pose directly.
   * Useful for initializing simulation to a known position.
   *
   * @param pose the desired robot pose in simulation world coordinates
   */
  public void setSimulationPose(Pose2d pose) {
    m_driveSimulation.setSimulationWorldPose(pose);
  }

  /**
   * Sets the simulated robot speeds directly.
   * Useful for testing autonomous routines.
   *
   * @param speeds the desired chassis speeds, robot-relative
   */
  public void setSimulationSpeeds(ChassisSpeeds speeds) {
    m_driveSimulation.setRobotSpeeds(speeds);
  }
}