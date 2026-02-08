// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Spindexer subsystem using TalonFX with Kraken x60 motor
 * Simple spinner mechanism for indexing/conveying game pieces
 */
@Logged(name = "Spindexer")
public class Spindexer extends SubsystemBase {

  // Constants
  private final double spinnerSpeed = Constants.Spindexer.kSpinnerSpeed;
  private final int canID = Constants.Spindexer.kMotorCanID;
  private final double gearRatio = Constants.Spindexer.kGearRatio;
  private final double kP = Constants.Spindexer.kKP;
  private final double kI = Constants.Spindexer.kKI;
  private final double kD = Constants.Spindexer.kKD;
  private final double kS = Constants.Spindexer.kKS;
  private final double kV = Constants.Spindexer.kKV;
  private final double kA = Constants.Spindexer.kKA;
  private final boolean brakeMode = Constants.Spindexer.kBrakeMode;
  private final boolean enableStatorLimit = Constants.Spindexer.kEnableStatorLimit;
  private final int statorCurrentLimit = Constants.Spindexer.kStatorCurrentLimit;
  private final boolean enableSupplyLimit = Constants.Spindexer.kEnableSupplyLimit;
  private final double supplyCurrentLimit = Constants.Spindexer.kSupplyCurrentLimit;

  // Feedforward
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

  // Motor controller
  private final TalonFX motor;
  private final VelocityVoltage velocityRequest;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  /**
   * Creates a new Spindexer subsystem.
   */
  public Spindexer() {
    // Initialize motor controller
    motor = new TalonFX(canID);

    // Create control requests
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    // Get status signals
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    // Configure motor
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
  }

  /**
   * Update telemetry signals.
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

  //------------------------ Motor Methods -----------------------//
  
  /**
   * Get the current position in rotations.
   * @return Position in rotations
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
   * Get the target velocity from the current velocity request.
   * @return Target velocity in rotations per second
   */
  @Logged(name = "Target Velocity/RotationsPerSecond")
  public double getTargetVelocity() {
    return velocityRequest.Velocity;
  }

  /**
   * Get the current applied voltage.
   * @return Applied voltage in volts
   */
  @Logged(name = "Voltage")
  public double getVoltage() {
    return voltageSignal.getValueAsDouble();
  }

  /**
   * Get the current motor current.
   * @return Motor current in amps
   */
  @Logged(name = "Current")
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
   */
  @Logged(name = "Temperature")
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }

  /**
   * Set motor velocity.
   * @param velocityRotSec The target velocity in rotations per second
   */
  private void setVelocity(double velocityRotSec) {
    setVelocity(velocityRotSec, 0);
  }

  /**
   * Set motor velocity with acceleration.
   * @param velocityRotSec The target velocity in rotations per second
   * @param acceleration The acceleration in rotations per second squared
   */
  private void setVelocity(double velocityRotSec, double acceleration) {
    double ffVolts = feedforward.calculate(velocityRotSec);
    motor.setControl(velocityRequest.withVelocity(velocityRotSec).withFeedForward(ffVolts));
  }

  /**
   * Set motor voltage directly.
   * @param voltage The voltage to apply
   */
  private void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Stop the motor.
   */
  private void stop() {
    setVelocity(0);
  }

  /**
   * Spin at the default speed.
   */
  private void spin() {
    setVelocity(spinnerSpeed);
  }

  /**
   * Spin in reverse at the default speed.
   */
  private void spinReverse() {
    setVelocity(-spinnerSpeed);
  }

  //------------------------ Command Methods -----------------------//

  /**
   * Creates a command to stop the spindexer.
   * @return A command that stops the motor
   */
  public Command stopCommand() {
    return runOnce(() -> stop());
  }

  /**
   * Creates a command to spin the spindexer at default speed.
   * @return A command that spins the motor at default speed
   */
  public Command spinCommand() {
    return run(() -> spin());
  }

  /**
   * Creates a command to spin the spindexer in reverse at default speed.
   * @return A command that spins the motor in reverse at default speed
   */
  public Command spinReverseCommand() {
    return run(() -> spinReverse());
  }

  /**
   * Creates a command to spin the spindexer at a specific velocity.
   * @param velocityRotSec The target velocity in rotations per second
   * @return A command that spins the motor at the specified velocity
   */
  public Command spinAtVelocityCommand(double velocityRotSec) {
    return run(() -> setVelocity(velocityRotSec));
  }

  /**
   * Creates a command to set the motor voltage directly.
   * @param voltage The voltage to apply
   * @return A command that applies the specified voltage
   */
  public Command setVoltageCommand(double voltage) {
    return run(() -> setVoltage(voltage));
  }
}
