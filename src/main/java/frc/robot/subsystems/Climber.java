package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.driverIO.DashboardPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Climber subsystem using TalonFX with Kraken x60 motor on a 15:1 gearbox spool.
 * Used for extending and retracting the climbing mechanism.
 */
@Logged(name = "Climber")
public class Climber extends SubsystemBase {

  // Constants
  private final int canID = Constants.Climber.kMotorCanID;
  private final double gearRatio = Constants.Climber.kGearRatio;
  private final double kP = Constants.Climber.kP;
  private final double kI = Constants.Climber.kI;
  private final double kD = Constants.Climber.kD;
  private final double kS = Constants.Climber.kS;
  private final double kV = Constants.Climber.kV;
  private final double kA = Constants.Climber.kA;
  private final boolean brakeMode = Constants.Climber.kBrakeMode;

  private final boolean enableStatorLimit = Constants.Climber.kEnableStatorLimit;
  private final int statorCurrentLimit = Constants.Climber.kStatorCurrentLimit;
  private final boolean enableSupplyLimit = Constants.Climber.kEnableSupplyLimit;
  private final double supplyCurrentLimit = Constants.Climber.kSupplyCurrentLimit;

  // Motor controller
  private final TalonFX motor;
  private final VelocityVoltage velocityRequest;
  private final MotionMagicVoltage positionRequest;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  // Simulation
  private final ElevatorSim climberSim;
  private final DCMotor dcMotor = DCMotor.getKrakenX60(1);

  // Feedforward
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, Constants.Climber.kG, kV, kA);

  // Target tracking for telemetry
  private double targetPositionRotations = 0.0;

  /**
   * Creates a new Climber Subsystem.
   */
  public Climber() {
    // Initialize motor controller
    motor = new TalonFX(canID);

    // Create control requests
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    positionRequest = new MotionMagicVoltage(0).withSlot(0);

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
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kG = Constants.Climber.kG;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;

    // Configure Motion Magic velocity/acceleration limits
    MotionMagicConfigs motionMagic = motorConfig.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = Constants.Climber.kMaxVelocity;
    motionMagic.MotionMagicAcceleration = Constants.Climber.kMaxAcceleration;

    // Set current limits
    CurrentLimitsConfigs currentLimits = motorConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = statorCurrentLimit;
    currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

    // Set soft limits
    SoftwareLimitSwitchConfigs softLimits = motorConfig.SoftwareLimitSwitch;
    softLimits.ForwardSoftLimitThreshold = Constants.Climber.kForwardSoftLimit;
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ReverseSoftLimitThreshold = Constants.Climber.kReverseSoftLimit;
    softLimits.ReverseSoftLimitEnable = true;

    // Set brake mode
    motorConfig.MotorOutput.NeutralMode = brakeMode
      ? NeutralModeValue.Brake
      : NeutralModeValue.Coast;

    // Set output direction (Update if inverted)
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Apply gear ratio
    motorConfig.Feedback.SensorToMechanismRatio = gearRatio;

    // Apply configuration
    motor.getConfigurator().apply(motorConfig);

    // Reset encoder position
    motor.setPosition(0);

    // Initialize simulation (convert rotations to meters for WPILib's internal ElevatorSim)
    double spoolCircumferenceMeters = Constants.Climber.kSpoolDiameterMeters * Math.PI;
    
    climberSim = new ElevatorSim(
      dcMotor,
      gearRatio,
      Constants.Climber.kSimCarriageMass,
      Constants.Climber.kSpoolDiameterMeters / 2.0, // radius
      Constants.Climber.kSimMinRotations * spoolCircumferenceMeters, // min height in meters
      Constants.Climber.kSimMaxRotations * spoolCircumferenceMeters, // max height in meters
      true, // simulate gravity
      0 // starting height
    );
  }

  @Override
  public void simulationPeriodic() {
    // Provide simulated battery voltage to the motor
    motor.getSimState().setSupplyVoltage(12.0);

    // Set input voltage from motor controller to simulation
    // We must pass the raw voltage out to WPILib sim
    climberSim.setInput(motor.getSimState().getMotorVoltage());

    // Update simulation by 20ms
    climberSim.update(0.020);

    // Update motor sim state
    double spoolRadius = Constants.Climber.kSpoolDiameterMeters / 2.0;
    double positionMeters = climberSim.getPositionMeters();
    double velocityMetersPerSec = climberSim.getVelocityMetersPerSecond();

    // Convert linear position/velocity to mechanism rotations
    // The physics simulation handles the gear ratio intrinsically, but Phoenix 
    // requires the *rotor* position and velocity since it applies the gear ratio internally.
    double mechanismRotations = positionMeters / (2.0 * Math.PI * spoolRadius);
    double mechanismRotationsPerSec = velocityMetersPerSec / (2.0 * Math.PI * spoolRadius);

    double rotorPosition = mechanismRotations * gearRatio;
    double rotorVelocity = mechanismRotationsPerSec * gearRatio;

    motor.getSimState().setRawRotorPosition(rotorPosition);
    motor.getSimState().setRotorVelocity(rotorVelocity);
  }

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

  //------------------- Basic getters -----------------//

  public ElevatorSim getSimulation() {
    return climberSim;
  }

  @Logged(name = "Position/Rotations")
  public double getPosition() {
    return positionSignal.getValueAsDouble();
  }

  @Logged(name = "Velocity/RPS")
  public double getVelocity() {
    return velocitySignal.getValueAsDouble();
  }

  @Logged(name = "Output Voltage")
  public double getVoltage() {
    return voltageSignal.getValueAsDouble();
  }

  @Logged(name = "Current/Amps")
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  @Logged(name = "Temperature/Celsius")
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }

  @Logged(name = "Target Position/Rotations")
  public double getTargetPosition() {
    return targetPositionRotations;
  }

  //------------------- Motor Setters -----------------//

  /**
   * Sets the velocity of the climber.
   * @param velocityRotSec The target velocity in rotations per second
   */
  private void setVelocity(double velocityRotSec) {
    motor.setControl(velocityRequest.withVelocity(velocityRotSec));
  }

  /**
   * Sets the position of the climber using Motion Magic.
   * @param positionRotations Target position in mechanism rotations.
   */
  private void setPosition(double positionRotations) {
    // Track target for telemetry
    this.targetPositionRotations = positionRotations;

    // We already use GravityType.Elevator_Static in Slot0 configs, 
    // so we don't need to manually pass feedforward unless desired.
    // double ffVolts = feedforward.calculate(getVelocity(), 0);
    motor.setControl(positionRequest.withPosition(positionRotations));
  }

  //------------------- Commands ----------------------//

  /**
   * Stops the climber motor.
   */
  public Command stopCommand() {
    return run(() -> motor.stopMotor());
  }

  /**
   * Manually drives the climber up.
   */
  public Command upCommand() {
    return run(() -> setVelocity(Constants.Climber.kManualUpSpeed));
  }

  /**
   * Manually drives the climber down.
   */
  public Command downCommand() {
    return run(() -> setVelocity(Constants.Climber.kManualDownSpeed));
  }

  /**
   * Command to extend the climber to the configured setpoint.
   */
  public Command extendCommand() {
    return run(() -> {
      setPosition(Constants.Climber.kExtendSetpointRotations);
    });
  }

  /**
   * Command to retract the climber to the configured setpoint.
   */
  public Command retractCommand() {
    return run(() -> {
      setPosition(Constants.Climber.kRetractSetpointRotations);
    });
  }

  /**
   * Resets the encoder position to zero.
   */
  public Command zeroEncoderCommand() {
    return runOnce(() -> motor.setPosition(0));
  }

}