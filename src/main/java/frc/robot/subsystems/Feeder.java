package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.driverIO.DashboardPublisher;

/**
 * Feeder subsystem using TalonFX with Krakenx60 motor
 * Used for feeding game pieces from intake to shooter
 */
@Logged(name = "Feeder")
public class Feeder extends SubsystemBase {

  // Constants
  private final double defaultSpeed = Constants.Feeder.kSpinnerSpeed;

  // Motor Constants
  private final int canID = Constants.Feeder.kMotorCanID;
  private final double gearRatio = Constants.Feeder.kGearRatio;
  private final double kP = Constants.Feeder.kKP;
  private final double kI = Constants.Feeder.kKI;
  private final double kD = Constants.Feeder.kKD;
  private final double kS = Constants.Feeder.kKS;
  private final double kV = Constants.Feeder.kKV;
  private final double kA = Constants.Feeder.kKA;
  private final boolean brakeMode = Constants.Feeder.kBrakeMode;
  private final boolean enableStatorLimit = Constants.Feeder.kEnableStatorLimit;
  private final int statorCurrentLimit = Constants.Feeder.kStatorCurrentLimit;
  private final boolean enableSupplyLimit = Constants.Feeder.kEnableSupplyLimit;
  private final double supplyCurrentLimit = Constants.Feeder.kSupplyCurrentLimit;

  // Feedforward
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    kS, // kS - static friction
    kV, // kV - velocity
    kA  // kA - acceleration
  );

  // Motor controller
  private final TalonFX motor;
  private final VelocityVoltage velocityRequest;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  // Simulation
  private final FlywheelSim feederSim;

  /**
   * Creates a new Feeder Subsystem.
   */
  public Feeder() {
    // Initialize motor controller
    motor = new TalonFX(canID);

    // Create control request
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    // Get status signals
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // Configure PID for slot 0
    Slot0Configs slot0 = motorConfig.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;

    // Set current limits
    CurrentLimitsConfigs currentLimits = motorConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = statorCurrentLimit;
    currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

    // Set brake mode
    motorConfig.MotorOutput.NeutralMode = brakeMode
      ? NeutralModeValue.Brake
      : NeutralModeValue.Coast;

    // Apply gear ratio
    motorConfig.Feedback.SensorToMechanismRatio = gearRatio;

    // Apply configuration
    motor.getConfigurator().apply(motorConfig);

    // Reset encoder position
    motor.setPosition(0);

    // Initialize simulation
    LinearSystem<N1, N1, N1> feederPlant = LinearSystemId.createFlywheelSystem(
      DCMotor.getKrakenX60(1),
      0.002, // Moment of inertia - rollers with game piece
      gearRatio
    );
    feederSim = new FlywheelSim(feederPlant, DCMotor.getKrakenX60(1));
  }

  /**
   * Update telemetry.
   */
  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
      positionSignal,
      velocitySignal,
      voltageSignal,
      statorCurrentSignal,
      temperatureSignal
    );
  }

  /**
   * Update simulation.
   */
  @Override
  public void simulationPeriodic() {
    // Set input voltage from motor controller to simulation
    feederSim.setInput(motor.getSimState().getMotorVoltage());
    
    // Update simulation by 20ms
    feederSim.update(0.020);

    // Update motor sim state
    double velocityRadPerSec = feederSim.getAngularVelocityRadPerSec();
    double motorVelocity = RadiansPerSecond.of(
      velocityRadPerSec * gearRatio
    ).in(RotationsPerSecond);

    motor.getSimState().setRotorVelocity(motorVelocity);
    
    // Update battery voltage
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        feederSim.getCurrentDrawAmps()
      )
    );
  }

  //------------------------ Motor Methods -----------------------//

  /**
   * Get the current position in Rotations.
   * @return Position in Rotations
   */
  @Logged(name = "Position/Rotations")
  public double getPosition() {
    return positionSignal.getValueAsDouble();
  }

  /**
   * Get the current velocity in rotations per second.
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity/RotationsPerSecond")
  public double getVelocity() {
    return velocitySignal.getValueAsDouble();
  }

  /**
   * Get the target velocity in rotations per second.
   * @return Target velocity in rotations per second
   */
  @Logged(name = "Target Velocity/RotationsPerSecond")
  public double getTargetVelocity() {
    return motor.getClosedLoopReference().getValueAsDouble();
  }

  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public double getVoltage() {
    return voltageSignal.getValueAsDouble();
  }

  /**
   * Get the current motor current.
   * @return Motor current in amps
   */
  @Logged(name = "Current/Amps")
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
   */
  @Logged(name = "Temperature/Celsius")
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }

  /**
   * Set motor angular velocity.
   * @param velocityRotSec The target velocity in rotations per second
   */
  private void setVelocity(double velocityRotSec) {
    setVelocity(velocityRotSec, 0);
  }

  /**
   * Set motor angular velocity with acceleration.
   * @param velocityRotSec The target velocity in rotations per second
   * @param acceleration The acceleration in rotations per second squared
   */
  private void setVelocity(double velocityRotSec, double acceleration) {
    motor.setControl(velocityRequest.withVelocity(velocityRotSec));
  }

  /**
   * Set motor voltage directly.
   * @param voltage The voltage to apply
   */
  private void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Get the feeder simulation for testing.
   * @return The feeder simulation model
   */
  public FlywheelSim getSimulation() {
    return feederSim;
  }

  /**
   * Creates a command to stop the feeder.
   * Uses run() (not runOnce) so the command never finishes and the subsystem
   * stays claimed as the default command.
   * @return A command that stops the feeder
   */
  public Command stopCommand() {
    return run(() -> {
      motor.stopMotor();
    });
  }

  /**
   * Creates a command to run the feeder at the default velocity.
   * @return A command that runs the feeder at default speed
   */
  public Command runCommand() {
    return run(() -> setVelocity(defaultSpeed));
  }

  /**
   * Creates a command to run the feeder at a specific velocity.
   * @param velocityRotSec The target velocity in rotations per second
   * @return A command that runs the feeder at the specified velocity
   */
  public Command runAtVelocityCommand(double velocityRotSec) {
    return run(() -> setVelocity(velocityRotSec));
  }

  /**
   * Creates a command to reverse the feeder (negative default speed).
   * @return A command that reverses the feeder
   */
  public Command reverseCommand() {
    return run(() -> setVelocity(-defaultSpeed));
  }

  /**
   * Check if feeder is running (velocity above threshold).
   * @return True if feeder is running
   */
  @Logged(name = "Is Running")
  public boolean isRunning() {
    return Math.abs(getVelocity()) > 1.0; // 1 RPS threshold
  }

  /**
   * Check if feeder is at target velocity (within tolerance).
   * @return True if at target velocity
   */
  @Logged(name = "At Target")
  public boolean atTargetVelocity() {
    double error = Math.abs(getTargetVelocity() - getVelocity());
    return error < 2.0; // 2 RPS tolerance
  }

  //------------------------ Tuning -----------------------//

  /**
   * Sets motor velocity using tunable PID values and setpoint from dashboard.
   * Updates PID gains in real-time and uses Setpoint1 for target velocity.
   */
  private void runTunable(DashboardPublisher dashboard) {
    if (dashboard == null) {
      System.err.println("Dashboard is null");
      return;
    }

    // Get tunable PID values
    double tunableKP = dashboard.getTunableKP();
    double tunableKI = dashboard.getTunableKI();
    double tunableKD = dashboard.getTunableKD();
    double tunableKV = dashboard.getTunableKV();
    double tunableKA = dashboard.getTunableKA();
    double setpoint = dashboard.getTunableSetpoint1();

    // Update PID gains in slot 0
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = tunableKP;
    slot0.kI = tunableKI;
    slot0.kD = tunableKD;
    slot0.kV = tunableKV;
    slot0.kA = tunableKA;
    slot0.kS = kS; // Keep original kS
    
    motor.getConfigurator().apply(slot0);

    // Set velocity using tunable setpoint
    setVelocity(setpoint);
  }

  /**
   * Command to tune feeder using dashboard values.
   * Continuously updates PID and setpoint from dashboard.
   */
  public Command tunableCommand(DashboardPublisher dashboard) {
    return run(() -> runTunable(dashboard));
  }
}
