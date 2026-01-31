package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * intake subsystem using TalonFX with Krakenx60 motor
 */
@Logged(name = "Intake")
public class Intake extends SubsystemBase {

  //Constants
  private final double spinnerSpeed = 10; // Target spinner speed in rotations per second
  private final double intakeAngleDeployed = 45; // Deployed angle in degrees
  private final double intakeAngleStowed = 0; // Stowed angle in degrees

  //Pivot Constants
  private final DCMotor dcMotorPivot = DCMotor.getKrakenX60(1);
  private final int canIDPivot = 10;
  private final double gearRatioPivot = 9;
  private final double PivotkP = 1;
  private final double PivotkI = 0;
  private final double PivotkD = 0;
  private final double PivotkS = 0;
  private final double PivotkV = 0;
  private final double PivotkA = 0;
  private final double PivotkG = 0; // Unused for intakes
  private final double PivotMaxVelocity = 1; // rad/s
  private final double PivotMaxAcceleration = 1; // rad/s²
  private final boolean IsPivotBrakeEnabled = true;
  private final double PivotForwardSoftLimit = 60; // max angle in radians
  private final double PivotReverseSoftLimit = 0; // min angle in radians
  private final boolean IsPivotStatorLimitEnabled = true;
  private final double PivotStatorCurrentLimit = 40;
  private final boolean IsPivotSupplyLimitEnabled = false;
  private final double PivotSupplyCurrentLimit = 40;

  
  // Constants for spinner
  private final DCMotor spinnerDcMotor = DCMotor.getNeo550(1);
  private final int spinnerCanID = 9;
  private final double spinnerGearRatio = 2;
  private final double spinnerKP = 1;
  private final double spinnerKI = 0;
  private final double spinnerKD = 0;
  private final double spinnerKS = 0;
  private final double spinnerKV = 0;
  private final double spinnerKA = 0;
  private final double spinnerKG = 0; // Unused for pivots
  private final double spinnerMaxVelocity = 1; // rad/s
  private final double spinnerMaxAcceleration = 1; // rad/s²
  private final boolean spinnerBrakeMode = true;
  private final double spinnerForwardSoftLimit = 0; // max angle in radians
  private final double spinnerReverseSoftLimit = 0; // min angle in radians
  private final boolean spinnerEnableStatorLimit = true;
  private final int spinnerStatorCurrentLimit = 40;
  private final boolean spinnerEnableSupplyLimit = false;
  private final double spinnerSupplyCurrentLimit = 40;
  

  // Feedforward
  private final ArmFeedforward Pivotfeedforward = new ArmFeedforward(
    PivotkS, // kS
    0, // kG - intake doesn't need gravity compensation
    PivotkV, // kV
    PivotkA // kA
  );

  // Feedforward for spinner
  private final ArmFeedforward spinnerFeedforward = new ArmFeedforward(
    spinnerKS, // kS
    0, // kG - spinner doesn't need gravity compensation
    spinnerKV, // kV
    spinnerKA // kA
  );

  

  //pivot Motor controller
  private final TalonFX PivotMotor;
  private final PositionVoltage pivotPositionRequest;
  private final VelocityVoltage pivotVelocityRequest;
  private final StatusSignal<Angle> pivotPositionSignal;
  private final StatusSignal<AngularVelocity> pivotVelocitySignal;
  private final StatusSignal<Voltage> pivotVoltageSignal;
  private final StatusSignal<Current> pivotStatorCurrentSignal;
  private final StatusSignal<Temperature> pivotTemperatureSignal;

  //Spinner Motor controller
  private final SparkMax SpinnerMotor;
  private final RelativeEncoder SpinnerEncoder;
  private final SparkSim SpinnerMotorSim;
  private final SparkClosedLoopController sparkPidController;
  

  // Simulation
  private final SingleJointedArmSim intakeSim;

  /**
   * Creates a new intake Subsystem.
   */
  public Intake() {
    // Initialize pivot motor controller
    PivotMotor = new TalonFX(canIDPivot);

    // Initialize spinner motor controller
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    SpinnerMotor = new SparkMax(spinnerCanID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    motorConfig.idleMode(spinnerBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);


    // Configure spinner encoder
    SpinnerEncoder = SpinnerMotor.getEncoder();
    SpinnerEncoder.setPosition(0);

    //pivot Create control requests
    pivotPositionRequest = new PositionVoltage(0).withSlot(0);
    pivotVelocityRequest = new VelocityVoltage(0).withSlot(0);

    // pivot get status signals
    pivotPositionSignal = PivotMotor.getPosition();
    pivotVelocitySignal = PivotMotor.getVelocity();
    pivotVoltageSignal = PivotMotor.getMotorVoltage();
    pivotStatorCurrentSignal = PivotMotor.getStatorCurrent();
    pivotTemperatureSignal = PivotMotor.getDeviceTemp();

    TalonFXConfiguration PivotMotorConfig = new TalonFXConfiguration();

    //Pivot Configure PID for slot 0
    Slot0Configs slot0 = PivotMotorConfig.Slot0;
    slot0.kP = PivotkP;
    slot0.kI = PivotkI;
    slot0.kD = PivotkD;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kS = PivotkS;
    slot0.kV = PivotkV;
    slot0.kA = PivotkA;

    // Spinner Configure Feedback and Feedforward
    sparkPidController = SpinnerMotor.getClosedLoopController();
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(spinnerKP, spinnerKI, spinnerKD, ClosedLoopSlot.kSlot0);
    motorConfig.closedLoop.feedForward.kS(spinnerKS).kV(spinnerKV).kA(spinnerKA);
    motorConfig.closedLoop.feedForward.kG(spinnerKG);

    // Pivot Set current limits
    CurrentLimitsConfigs currentLimits = PivotMotorConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = PivotStatorCurrentLimit;
    currentLimits.StatorCurrentLimitEnable = IsPivotStatorLimitEnabled;
    currentLimits.SupplyCurrentLimit = PivotSupplyCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = IsPivotSupplyLimitEnabled;

    // Set spinner current limits
    motorConfig.smartCurrentLimit(spinnerStatorCurrentLimit);

    // pivot Set soft limits
    SoftwareLimitSwitchConfigs softLimits = PivotMotorConfig.SoftwareLimitSwitch;
    softLimits.ForwardSoftLimitThreshold = PivotForwardSoftLimit;
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ReverseSoftLimitThreshold = PivotReverseSoftLimit;
    softLimits.ReverseSoftLimitEnable = true;

    // pivot Set brake mode
    PivotMotorConfig.MotorOutput.NeutralMode = IsPivotBrakeEnabled
      ? NeutralModeValue.Brake
      : NeutralModeValue.Coast;

    // pivot Apply gear ratio
    PivotMotorConfig.Feedback.SensorToMechanismRatio = gearRatioPivot;

        // Spinner Configure Encoder Gear Ratio
    motorConfig.encoder
      .positionConversionFactor(1 / spinnerGearRatio)
      .velocityConversionFactor((1 / spinnerGearRatio) / 60); // Covnert RPM to RPS

    // pivot Apply configuration
    PivotMotor.getConfigurator().apply(PivotMotorConfig);

    //Spinner Save configuration
    SpinnerMotor.configure(
      motorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    SpinnerMotorSim = new SparkSim(SpinnerMotor, spinnerDcMotor);

    // pivot Reset encoder position
    PivotMotor.setPosition(0);

    // Initialize simulation
    intakeSim = new SingleJointedArmSim(
      dcMotorPivot, // Motor type
      gearRatioPivot,
      0.01, // Arm moment of inertia - Small value since there are no arm parameters
      0.1, // Arm length (m) - Small value since there are no arm parameters
      Units.degreesToRadians(-90), // Min angle (rad)
      Units.degreesToRadians(90), // Max angle (rad)
      false, // Simulate gravity - Disable gravity for intake
      Units.degreesToRadians(0) // Starting position (rad)
    );
  }

  /**
   * Update simulation and telemetry.
   */
  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
      pivotPositionSignal,
      pivotVelocitySignal,
      pivotVoltageSignal,
      pivotStatorCurrentSignal,
      pivotTemperatureSignal
    );
  }

  /**
   * Update simulation.
   */
  @Override
  public void simulationPeriodic() {
    // Set input voltage from motor controller to simulation
    // Use motor voltage for TalonFX simulation input
    intakeSim.setInput(PivotMotor.getSimState().getMotorVoltage());

    // Update simulation by 20ms
    intakeSim.update(0.020);
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        intakeSim.getCurrentDrawAmps()
      )
    );

    double motorPosition = Radians.of(intakeSim.getAngleRads() * gearRatioPivot).in(
      Rotations
    );
    double motorVelocity = RadiansPerSecond.of(
      intakeSim.getVelocityRadPerSec() * gearRatioPivot
    ).in(RotationsPerSecond);

    PivotMotor.getSimState().setRawRotorPosition(motorPosition);
    PivotMotor.getSimState().setRotorVelocity(motorVelocity);
  }


  //------------------------ Pivot Methods -----------------------//
  /**
   * Get the current position in Rotations.
   * @return Position in Rotations
   */
  @Logged(name = "Pivot Position/Rotations")
  public double PivotGetPosition() {
    // Rotations
    return pivotPositionSignal.getValueAsDouble();
  }

  /**
   * Get the current position in Degrees.
   * @return Position in Degrees.
   */
  @Logged(name = "Pivot Position/Degrees")
  public double PivotGetPositionDegrees() {
    return PivotGetPosition() * 360;
  }

  /**
   * Get the current velocity in rotations per second.
   * @return Velocity in rotations per second
   */
  @Logged(name = "Pivot Velocity/RotationsPerSecond")
  public double PivotMotorGetVelocity() {
    return pivotVelocitySignal.getValueAsDouble();
  }

  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
  @Logged(name = "Pivot Voltage")
  public double PivotMotorGetVoltage() {
    return pivotVoltageSignal.getValueAsDouble();
  }

  /**
   * Get the current motor current.
   * @return Motor current in amps
   */
  public double PivotMotorGetCurrent() {
    return pivotStatorCurrentSignal.getValueAsDouble();
  }

  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
   */
  public double PivotMotorgetTemperature() {
    return pivotTemperatureSignal.getValueAsDouble();
  }

  /**
   * Set intake angle.
   * @param angleDegrees The target angle in degrees
   */
  private void PivotSetAngle(double angleDegrees) {
    PivotSetAngle(angleDegrees, 0);
  }

  /**
   * Set intake pivot with acceleration.
   * @param angleDegrees The target angle in degrees
   * @param acceleration The acceleration in rad/s²
   */
  private void PivotSetAngle(double angleDegrees, double acceleration) {
    // Convert degrees to rotations
    double angleRadians = Units.degreesToRadians(angleDegrees);
    double positionRotations = angleRadians / (2.0 * Math.PI);

    double PivotffVolts = Pivotfeedforward.calculate(PivotMotorGetVelocity(), acceleration);
    //motor.setControl(positionRequest.withPosition(positionRotations).withFeedForward(ffVolts));
    PivotMotor.setControl(pivotPositionRequest.withPosition(positionRotations));
  }

  /**
   * Set intake angular velocity.
   * @param velocityDegPerSec The target velocity in degrees per second
   */
  private void PivotSetVelocity(double velocityDegPerSec) {
    PivotSetVelocity(velocityDegPerSec, 0);
  }

  /**
   * Set intake angular velocity with acceleration.
   * @param velocityDegPerSec The target velocity in degrees per second
   * @param acceleration The acceleration in degrees per second squared
   */
  private void PivotSetVelocity(double velocityDegPerSec, double acceleration) {
    // Convert degrees/sec to rotations/sec
    double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
    double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);

    double ffVolts = Pivotfeedforward.calculate(PivotMotorGetVelocity(), acceleration);
    //motor.setControl(velocityRequest.withVelocity(velocityRotations).withFeedForward(ffVolts));
    PivotMotor.setControl(pivotVelocityRequest.withVelocity(velocityRotations));
  }

  /**
   * Set motor voltage directly.
   * @param voltage The voltage to apply
   */
  private void PivotSetVoltage(double voltage) {
    PivotMotor.setVoltage(voltage);
  }

  /**
   * Get the intake simulation for testing.
   * @return The intake simulation model
   */
  public SingleJointedArmSim getSimulation() {
    return intakeSim;
  }
  /**
   * Creates a command to stop the intake.
   * @return A command that stops the intake
   */
  public Command PivotStopCommand() {
    return runOnce(() -> PivotSetVelocity(0));
  }

  public Command PivotSetAngleCommand(double angleDegrees) {
    return runOnce(() -> PivotSetAngle(angleDegrees));
  }

  //------------------------ Spinner Methods -----------------------//
  /**
   * Get the current position in Rotations.
   * @return Position in Rotations
   */
  @Logged(name = "Spinner Position/Rotations")
  public double SpinnerGetPosition() {
    // Rotations
    return SpinnerEncoder.getPosition();
  }

  /**
   * Get the current velocity in rotations per second.
   * @return Velocity in rotations per second
   */
  @Logged(name = "Spinner Velocity/RotationsPerSecond")
  public double SpinnerGetVelocity() {
    return SpinnerEncoder.getVelocity();
  }

  @Logged(name = "Target Spinner Velocity/RotationsPerSecond")
  public double SpinnerGetTargetVelocity() {
    return sparkPidController.getSetpoint();
  }

  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
  @Logged(name = "Spinner Voltage")
  public double SpinnerGetVoltage() {
    return SpinnerMotor.getAppliedOutput() * SpinnerMotor.getBusVoltage();
  }


  /**
   * Get the current motor current.
   * @return Motor current in amps
   */
  public double SpinnerGetCurrent() {
    return SpinnerMotor.getOutputCurrent();
  }

  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
   */
  public double SpinnerGetTemperature() {
    return SpinnerMotor.getMotorTemperature();
  }

  /**
   * Set spinner angular velocity.
   * @param velocityRotSec The target velocity in rotations per second
   */
  private void SpinnerSetVelocity(double velocityRotSec) {
    SpinnerSetVelocity(velocityRotSec, 0);
  }


  /**
   * Set spinner angular velocity with acceleration.
   * @param velocityRotSec The target velocity in rotations per second
   * @param acceleration The acceleration in rotations per second squared
   */
  private void SpinnerSetVelocity(double velocityRotSec, double acceleration) {

    sparkPidController.setSetpoint(
      velocityRotSec,
      ControlType.kVelocity,
      ClosedLoopSlot.kSlot0
    );
  }

  /**
   * Set motor voltage directly.
   * @param voltage The voltage to apply
   */
  private void SpinnerSetVoltage(double voltage) {
    SpinnerMotor.setVoltage(voltage);
  }

  /**
   * Creates a command to stop the spinner.
   * @return A command that stops the spinner
   */
  public Command SpinnerStopCommand() {
    return runOnce(() -> SpinnerSetVelocity(0));
  }

  /**
   * Creates a command to move the spinner at a specific velocity.
   * @param velocityRotSec The target velocity in rotations per second
   * @return A command that moves the spinner at the specified velocity
   */
  public Command SpinnerMoveAtVelocityCommand(double velocityRotSec) {
    return run(() -> SpinnerSetVelocity(velocityRotSec));
  }

  //------------------------ Useful Methods -----------------------//

  private void stow(){
    PivotSetAngle(intakeAngleStowed);
    SpinnerSetVelocity(0);
  }

  public Command StowCommand() {
    return runOnce(() -> stow());
  }

  public void deploy(){
    PivotSetAngle(intakeAngleDeployed);
    SpinnerSetVelocity(spinnerSpeed);
  }
  public Command DeployCommand() {
    return runOnce(() -> deploy());
  }
  
}
