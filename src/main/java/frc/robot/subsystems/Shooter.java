package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import frc.robot.Constants;
import frc.robot.driverIO.DashboardPublisher;
import frc.robot.utils.TurretUtil;

/**
 * Shooter subsystem with dual Kraken X60 flywheels and NEO 550 adjustable hood
 */
@Logged(name = "Shooter")
public class Shooter extends SubsystemBase {

  // Flywheel Constants
  private final double defaultFlywheelSpeed = Constants.Shooter.kDefaultFlywheelSpeed;
  private final int leftFlywheelCanID = Constants.Shooter.kLeftFlywheelCanID;
  private final int rightFlywheelCanID = Constants.Shooter.kRightFlywheelCanID;
  private final double flywheelGearRatio = Constants.Shooter.kFlywheelGearRatio;
  private final double flywheelKP = Constants.Shooter.kFlywheelKP;
  private final double flywheelKI = Constants.Shooter.kFlywheelKI;
  private final double flywheelKD = Constants.Shooter.kFlywheelKD;
  private final double flywheelKS = Constants.Shooter.kFlywheelKS;
  private final double flywheelKV = Constants.Shooter.kFlywheelKV;
  private final double flywheelKA = Constants.Shooter.kFlywheelKA;

  // Hood Constants
  private final int hoodCanID = Constants.Shooter.kHoodCanID;
  private final double hoodGearRatio = Constants.Shooter.kHoodGearRatio;
  private final double hoodMinAngle = Constants.Shooter.kHoodMinAngleDegrees;
  private final double hoodMaxAngle = Constants.Shooter.kHoodMaxAngleDegrees;
  private final double hoodKP = Constants.Shooter.kHoodKP;
  private final double hoodKI = Constants.Shooter.kHoodKI;
  private final double hoodKD = Constants.Shooter.kHoodKD;
  private final double hoodMaxVelocity = Constants.Shooter.kHoodMaxVelocity;
  private final double hoodMaxAcceleration = Constants.Shooter.kHoodMaxAcceleration;

  // Flywheel Motors
  private final TalonFX leftFlywheel;
  private final TalonFX rightFlywheel;
  private final VelocityVoltage velocityRequest;
  private final StatusSignal<AngularVelocity> leftVelocitySignal;
  private final StatusSignal<AngularVelocity> rightVelocitySignal;
  private final StatusSignal<Voltage> leftVoltageSignal;
  private final StatusSignal<Current> leftCurrentSignal;
  private final StatusSignal<Temperature> leftTempSignal;

  // Hood Motor
  private final SparkMax hoodMotor;
  private final RelativeEncoder hoodEncoder;
  private final SparkClosedLoopController hoodController;
  private double targetHoodAngle = Constants.Shooter.kHoodStartAngleDegrees;

  // Simulation
  private final FlywheelSim leftFlywheelSim;
  private final FlywheelSim rightFlywheelSim;
  private final SingleJointedArmSim hoodSim;
  private final PIDController hoodSimPID;

  /**
   * Creates a new Shooter Subsystem.
   */
  public Shooter() {
    // Initialize flywheel motors
    leftFlywheel = new TalonFX(leftFlywheelCanID);
    rightFlywheel = new TalonFX(rightFlywheelCanID);

    // Configure left flywheel (leader)
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    Slot0Configs leftSlot0 = leftConfig.Slot0;
    leftSlot0.kP = flywheelKP;
    leftSlot0.kI = flywheelKI;
    leftSlot0.kD = flywheelKD;
    leftSlot0.kS = flywheelKS;
    leftSlot0.kV = flywheelKV;
    leftSlot0.kA = flywheelKA;

    CurrentLimitsConfigs leftCurrentLimits = leftConfig.CurrentLimits;
    leftCurrentLimits.StatorCurrentLimit = Constants.Shooter.kFlywheelStatorCurrentLimit;
    leftCurrentLimits.StatorCurrentLimitEnable = Constants.Shooter.kFlywheelEnableStatorLimit;
    leftCurrentLimits.SupplyCurrentLimit = Constants.Shooter.kFlywheelSupplyCurrentLimit;
    leftCurrentLimits.SupplyCurrentLimitEnable = Constants.Shooter.kFlywheelEnableSupplyLimit;

    leftConfig.MotorOutput.NeutralMode = Constants.Shooter.kFlywheelBrakeMode
      ? NeutralModeValue.Brake
      : NeutralModeValue.Coast;

    leftConfig.Feedback.SensorToMechanismRatio = flywheelGearRatio;
    leftFlywheel.getConfigurator().apply(leftConfig);

    // Configure right flywheel (mirror of left, with inverted direction for counter-rotation)
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    
    // Copy PID settings to right motor
    Slot0Configs rightSlot0 = rightConfig.Slot0;
    rightSlot0.kP = flywheelKP;
    rightSlot0.kI = flywheelKI;
    rightSlot0.kD = flywheelKD;
    rightSlot0.kS = flywheelKS;
    rightSlot0.kV = flywheelKV;
    rightSlot0.kA = flywheelKA;

    // Copy current limits
    CurrentLimitsConfigs rightCurrentLimits = rightConfig.CurrentLimits;
    rightCurrentLimits.StatorCurrentLimit = Constants.Shooter.kFlywheelStatorCurrentLimit;
    rightCurrentLimits.StatorCurrentLimitEnable = Constants.Shooter.kFlywheelEnableStatorLimit;
    rightCurrentLimits.SupplyCurrentLimit = Constants.Shooter.kFlywheelSupplyCurrentLimit;
    rightCurrentLimits.SupplyCurrentLimitEnable = Constants.Shooter.kFlywheelEnableSupplyLimit;

    rightConfig.MotorOutput.NeutralMode = Constants.Shooter.kFlywheelBrakeMode
      ? NeutralModeValue.Brake
      : NeutralModeValue.Coast;
    
    // Invert right motor for counter-rotation
    rightConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
    rightConfig.Feedback.SensorToMechanismRatio = flywheelGearRatio;
    rightFlywheel.getConfigurator().apply(rightConfig);

    // Create velocity request for both motors
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    // Get status signals
    leftVelocitySignal = leftFlywheel.getVelocity();
    rightVelocitySignal = rightFlywheel.getVelocity();
    leftVoltageSignal = leftFlywheel.getMotorVoltage();
    leftCurrentSignal = leftFlywheel.getStatorCurrent();
    leftTempSignal = leftFlywheel.getDeviceTemp();

    // Initialize hood motor (NEO 550 on SparkMax)
    hoodMotor = new SparkMax(hoodCanID, MotorType.kBrushless);
    hoodEncoder = hoodMotor.getEncoder();
    hoodController = hoodMotor.getClosedLoopController();

    // Configure hood - using simplified API
    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    hoodConfig.idleMode(Constants.Shooter.kHoodBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    hoodConfig.smartCurrentLimit(Constants.Shooter.kHoodCurrentLimit);
    
    // PID configuration
    hoodConfig.closedLoop.pid(hoodKP, hoodKI, hoodKD);

    // MAXMotion (motion profiling) configuration
    // cruiseVelocity is in motor RPM, maxAcceleration is in motor RPM/s
    // Convert mechanism degrees/s → motor RPM:  (deg/s) / 360 * gearRatio * 60
    double hoodCruiseVelocityRPM = hoodMaxVelocity / 360.0 * hoodGearRatio * 60.0;
    double hoodMaxAccelRPMPerSec = hoodMaxAcceleration / 360.0 * hoodGearRatio * 60.0;
    hoodConfig.closedLoop.maxMotion
        .cruiseVelocity(hoodCruiseVelocityRPM)
        .maxAcceleration(hoodMaxAccelRPMPerSec)
        .allowedProfileError(1.0 / 360.0 * hoodGearRatio); // 1 degree tolerance in motor rotations
    
    // Soft limits
    hoodConfig.softLimit.forwardSoftLimit(hoodMaxAngle / 360.0 * hoodGearRatio);
    hoodConfig.softLimit.forwardSoftLimitEnabled(true);
    hoodConfig.softLimit.reverseSoftLimit(hoodMinAngle / 360.0 * hoodGearRatio);
    hoodConfig.softLimit.reverseSoftLimitEnabled(true);
    
    // Apply configuration (simplified - no reset/persist mode needed in 2026 API)
    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

    // Set hood starting position (assume it starts at 90 degrees)
    hoodEncoder.setPosition(Constants.Shooter.kHoodStartAngleDegrees / 360.0 * hoodGearRatio);

    // Initialize simulations
    LinearSystem<N1, N1, N1> leftFlywheelPlant = LinearSystemId.createFlywheelSystem(
      DCMotor.getKrakenX60(1),
      Constants.Shooter.kSimFlywheelMomentOfInertia,
      flywheelGearRatio
    );
    leftFlywheelSim = new FlywheelSim(leftFlywheelPlant, DCMotor.getKrakenX60(1));

    LinearSystem<N1, N1, N1> rightFlywheelPlant = LinearSystemId.createFlywheelSystem(
      DCMotor.getKrakenX60(1),
      Constants.Shooter.kSimFlywheelMomentOfInertia,
      flywheelGearRatio
    );
    rightFlywheelSim = new FlywheelSim(rightFlywheelPlant, DCMotor.getKrakenX60(1));

    hoodSim = new SingleJointedArmSim(
      DCMotor.getNeo550(1),
      hoodGearRatio,
      Constants.Shooter.kSimHoodMomentOfInertia,
      Constants.Shooter.kSimHoodLength,
      Units.degreesToRadians(hoodMinAngle),
      Units.degreesToRadians(hoodMaxAngle),
      true, // Simulate gravity
      Units.degreesToRadians(Constants.Shooter.kHoodStartAngleDegrees)
    );

    // PID controller for hood simulation (matches SparkMax PID settings)
    // Note: WPILib PID works in units, not rotations, so we need to scale
    // The hood position is in degrees, so PID needs to be scaled accordingly
    hoodSimPID = new PIDController(
      hoodKP / hoodGearRatio, //* 360.0, // Scale from rotations to degrees
      hoodKI / hoodGearRatio, //* 360.0,
      hoodKD / hoodGearRatio //* 360.0
    );
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
      leftVelocitySignal,
      rightVelocitySignal,
      leftVoltageSignal,
      leftCurrentSignal,
      leftTempSignal
    );
  }

  @Override
  public void simulationPeriodic() {
    // Simulate flywheels
    leftFlywheelSim.setInput(leftFlywheel.getSimState().getMotorVoltage());
    leftFlywheelSim.update(0.020);
    
    rightFlywheelSim.setInput(rightFlywheel.getSimState().getMotorVoltage());
    rightFlywheelSim.update(0.020);

    double leftVelocity = RadiansPerSecond.of(
      leftFlywheelSim.getAngularVelocityRadPerSec() * flywheelGearRatio
    ).in(RotationsPerSecond);
    
    double rightVelocity = RadiansPerSecond.of(
      rightFlywheelSim.getAngularVelocityRadPerSec() * flywheelGearRatio
    ).in(RotationsPerSecond);

    leftFlywheel.getSimState().setRotorVelocity(leftVelocity);
    rightFlywheel.getSimState().setRotorVelocity(rightVelocity);

    // Simulate hood
    // SparkMax doesn't have native simulation support like TalonFX
    // We need to manually calculate PID output for simulation
    double currentHoodAngleDeg = Units.radiansToDegrees(hoodSim.getAngleRads());
    double pidOutput = hoodSimPID.calculate(currentHoodAngleDeg, targetHoodAngle);
    
    // Clamp output to -12V to +12V
    double batteryVoltage = RoboRioSim.getVInVoltage();
    if (batteryVoltage < 6.0) batteryVoltage = 12.0; // Default if not set
    double hoodVoltage = Math.max(-batteryVoltage, Math.min(batteryVoltage, pidOutput));
    
    hoodSim.setInput(hoodVoltage);
    hoodSim.update(0.020);

    // Update hood encoder with simulated position
    double hoodAngleRadians = hoodSim.getAngleRads();
    double hoodAngleDegrees = Units.radiansToDegrees(hoodAngleRadians);
    double hoodPositionRotations = hoodAngleDegrees / 360.0 * hoodGearRatio;
    hoodEncoder.setPosition(hoodPositionRotations);

    // Debug output for hood simulation
    if (Math.abs(targetHoodAngle - hoodAngleDegrees) > 1.0) {
      System.out.println(String.format(
        "Hood Sim - Target: %.1f deg, Actual: %.1f deg, PID Output: %.2f V, Voltage: %.2f V, Battery: %.2f V",
        targetHoodAngle, hoodAngleDegrees, pidOutput, hoodVoltage, batteryVoltage
      ));
    }

    // Update battery voltage
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        leftFlywheelSim.getCurrentDrawAmps() + 
        rightFlywheelSim.getCurrentDrawAmps() + 
        hoodSim.getCurrentDrawAmps()
      )
    );
  }

  //------------------------ Flywheel Methods -----------------------//

  /**
   * Get the left flywheel velocity in RPS.
   */
  @Logged(name = "Flywheel/Left Velocity RPS")
  public double getLeftFlywheelVelocity() {
    return leftVelocitySignal.getValueAsDouble();
  }

  /**
   * Get the right flywheel velocity in RPS.
   */
  @Logged(name = "Flywheel/Right Velocity RPS")
  public double getRightFlywheelVelocity() {
    return rightVelocitySignal.getValueAsDouble();
  }

  /**
   * Get the average flywheel velocity in RPS.
   */
  @Logged(name = "Flywheel/Average Velocity RPS")
  public double getAverageFlywheelVelocity() {
    return (getLeftFlywheelVelocity() + Math.abs(getRightFlywheelVelocity())) / 2.0;
  }

  /**
   * Get the target flywheel velocity in RPS.
   */
  @Logged(name = "Flywheel/Target Velocity RPS")
  public double getTargetFlywheelVelocity() {
    return leftFlywheel.getClosedLoopReference().getValueAsDouble();
  }

  /**
   * Get flywheel voltage.
   */
  @Logged(name = "Flywheel/Voltage")
  public double getFlywheelVoltage() {
    return leftVoltageSignal.getValueAsDouble();
  }

  /**
   * Get flywheel current.
   */
  @Logged(name = "Flywheel/Current")
  public double getFlywheelCurrent() {
    return leftCurrentSignal.getValueAsDouble();
  }

  /**
   * Get flywheel temperature.
   */
  @Logged(name = "Flywheel/Temperature")
  public double getFlywheelTemperature() {
    return leftTempSignal.getValueAsDouble();
  }

  /**
   * Check if flywheel is at target speed.
   */
  @Logged(name = "Flywheel/At Speed")
  public boolean isFlywheelAtSpeed() {
    double error = Math.abs(getTargetFlywheelVelocity() - getAverageFlywheelVelocity());
    return error < 2.0; // 2 RPS tolerance
  }

  /**
   * Set flywheel velocity (controls both motors with same speed, inverted via configuration).
   */
  private void setFlywheelVelocity(double velocityRPS) {
    leftFlywheel.setControl(velocityRequest.withVelocity(velocityRPS));
    rightFlywheel.setControl(velocityRequest.withVelocity(velocityRPS)); // Right is inverted in config
  }

  /**
   * Stop flywheels.
   */
  private void stopFlywheels() {
    leftFlywheel.stopMotor();
    rightFlywheel.stopMotor();
  }

  //------------------------ Hood Methods -----------------------//

  /**
   * Get hood angle in degrees.
   */
  @Logged(name = "Hood/Angle Degrees")
  public double getHoodAngle() {
    return hoodEncoder.getPosition() / hoodGearRatio * 360.0;
  }

  /**
   * Get hood velocity in degrees per second.
   */
  @Logged(name = "Hood/Velocity Deg per Sec")
  public double getHoodVelocity() {
    return hoodEncoder.getVelocity() / hoodGearRatio * 360.0 / 60.0; // RPM to deg/s
  }

  /**
   * Set hood angle in degrees.
   */
  private void setHoodAngle(double angleDegrees) {
    targetHoodAngle = angleDegrees;
    double positionRotations = angleDegrees / 360.0 * hoodGearRatio;
    hoodController.setSetpoint(positionRotations, ControlType.kMAXMotionPositionControl);
  }

  /**
   * Get target hood angle in degrees.
   */
  @Logged(name = "Hood/Target Angle Degrees")
  public double getTargetHoodAngle() {
    return targetHoodAngle;
  }

  /**
   * Check if hood is at target angle.
   */
  @Logged(name = "Hood/At Target")
  public boolean isHoodAtTarget() {
    // Get target from closed loop controller - for now just check velocity is low
    return Math.abs(getHoodVelocity()) < 5.0; // 5 deg/s tolerance
  }

  //------------------------ Commands -----------------------//

  /**
   * Stop shooter (flywheels and hood).
   */
  public Command stopCommand() {
    return runOnce(() -> {
      stopFlywheels();
      hoodMotor.stopMotor();
    });
  }

  /**
   * Run flywheels at default speed.
   */
  public Command runFlywheelsCommand() {
    return run(() -> setFlywheelVelocity(defaultFlywheelSpeed));
  }

  /**
   * Run flywheels at specific speed.
   */
  public Command runFlywheelsAtSpeedCommand(double velocityRPS) {
    return run(() -> setFlywheelVelocity(velocityRPS));
  }

  /**
   * Set hood to specific angle.
   */
  public Command setHoodAngleCommand(double angleDegrees) {
    return run(() -> setHoodAngle(angleDegrees));
  }

  /**
   * Set hood to 80 degrees to pass under ]trench
   * @param dashboard
   */
    public Command setHoodToTrenchCommand() {
      return run(() -> setHoodAngle(Constants.Shooter.kHoodTrenchAngleDegrees));
    }

  //------------------------ Tuning -----------------------//

  /**
   * Tunable command for flywheels.
   */
  private void runFlywheelTunable(DashboardPublisher dashboard) {
    if (dashboard == null) return;

    double tunableKP = dashboard.getTunableKP();
    double tunableKI = dashboard.getTunableKI();
    double tunableKD = dashboard.getTunableKD();
    double tunableKV = dashboard.getTunableKV();
    double tunableKA = dashboard.getTunableKA();
    double setpoint = dashboard.getTunableSetpoint1();

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = tunableKP;
    slot0.kI = tunableKI;
    slot0.kD = tunableKD;
    slot0.kV = tunableKV;
    slot0.kA = tunableKA;
    slot0.kS = flywheelKS;
    
    leftFlywheel.getConfigurator().apply(slot0);
    setFlywheelVelocity(setpoint);
  }

  /**
   * Command for tuning flywheels.
   */
  public Command flywheelTunableCommand(DashboardPublisher dashboard) {
    return run(() -> runFlywheelTunable(dashboard));
  }

  // Cached tunable hood kP — used to detect changes so we don't spam CAN bus every loop
  private double lastHoodTunableKP = Double.NaN;

  /**
   * Tunable command for hood.
   * Only re-applies PID config when kP changes to avoid flooding the CAN bus.
   */
  private void runHoodTunable(DashboardPublisher dashboard) {
    if (dashboard == null) return;

    double tunableKP = dashboard.getTunableKP();
    double setpoint = dashboard.getTunableSetpoint1();

    // Only reconfigure when kP actually changes
    if (tunableKP != lastHoodTunableKP) {
      lastHoodTunableKP = tunableKP;
      SparkMaxConfig config = new SparkMaxConfig();
      config.closedLoop.pid(tunableKP, hoodKI, hoodKD);
      hoodMotor.configure(config,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    }

    setHoodAngle(setpoint);
  }

  /**
   * Command for tuning hood.
   */
  public Command hoodTunableCommand(DashboardPublisher dashboard) {
    return run(() -> runHoodTunable(dashboard));
  }

  /**
   * "Shot tunable" command — simultaneously controls flywheel speed and hood angle
   * from the dashboard, updating both every loop.
   *
   * <p>Setpoint1 → flywheel speed (RPS)<br>
   * Setpoint2 → hood angle (degrees)
   *
   * <p>Uses suppliers so the values are read fresh on every execute() call rather
   * than being captured once at command construction time.
   *
   * @param speedSupplier     Supplier that returns the desired flywheel speed in RPS
   *                          (e.g. {@code dashboard::getTunableSetpoint1})
   * @param hoodAngleSupplier Supplier that returns the desired hood angle in degrees
   *                          (e.g. {@code dashboard::getTunableSetpoint2})
   * @return A command that continuously applies both setpoints while held
   */
  public Command shotTunableCommand(java.util.function.DoubleSupplier speedSupplier,
                                     java.util.function.DoubleSupplier hoodAngleSupplier) {
    return run(() -> {
      setFlywheelVelocity(speedSupplier.getAsDouble());
      setHoodAngle(hoodAngleSupplier.getAsDouble());
    }).withName("ShotTunable");
  }

  /**
   * Get flywheel sim for testing.
   */
  public FlywheelSim getLeftFlywheelSim() {
    return leftFlywheelSim;
  }

  /**
   * Get hood sim for testing.
   */
  public SingleJointedArmSim getHoodSim() {
    return hoodSim;
  }

  /**
   * Rezero the shooter hood encoder to 90 degrees.
   */
  private void rezero() {
    double rezeroPositionRotations = Constants.Shooter.kHoodRezeroAngleDegrees / 360.0 * hoodGearRatio;
    hoodEncoder.setPosition(rezeroPositionRotations);
  }

  /**
   * Command to rezero the shooter hood encoder.
   */
  public Command RezeroCommand() {
    return runOnce(() -> rezero());
  }

    public Command autoAimCommandShooter(Supplier<Pose2d> robotPoseSupplier, TurretUtil.TargetType target) {
    return run(() -> {
      Pose2d robotPose = robotPoseSupplier.get();
      TurretUtil.ShotSolution solution = TurretUtil.computeShotSolution(robotPose, target);
      
      if (solution.isValid) {
        setHoodAngle(solution.trajectoryAngleDegrees);
        setFlywheelVelocity(solution.shooterSpeedRPS);
      }
    }).withName("AutoAimShooter-" + target.toString());
  }

  /**
   * Continuously sets the hood angle and flywheel speed using the "shoot on the move"
   * iterative lead algorithm.
   *
   * @param robotPoseSupplier     Supplier for the current robot field pose
   * @param chassisSpeedsSupplier Supplier for the current field-relative chassis speeds
   * @param target                Which target to shoot at
   * @return A command that continuously updates hood and flywheel with motion compensation
   */
  public Command shootOnMoveCommandShooter(Supplier<Pose2d> robotPoseSupplier,
                                            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
                                            TurretUtil.TargetType target) {
    return run(() -> {
      Pose2d robotPose = robotPoseSupplier.get();
      ChassisSpeeds speeds = chassisSpeedsSupplier.get();
      TurretUtil.ShotSolution solution = TurretUtil.computeLeadShotSolution(
          robotPose, speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, target);

      if (solution.isValid) {
        setHoodAngle(solution.trajectoryAngleDegrees);
        setFlywheelVelocity(solution.shooterSpeedRPS);
      }
    }).withName("ShootOnMove-Shooter-" + target.toString());
  }
}
