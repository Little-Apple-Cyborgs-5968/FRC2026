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

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
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
import frc.robot.Constants;
import frc.robot.driverIO.DashboardPublisher;

/**
 * intake subsystem using TalonFX with Krakenx60 motors
 */
@Logged(name = "Intake")
public class Intake extends SubsystemBase {

  //Constants
  private final double spinnerSpeed = Constants.Intake.kSpinnerSpeed;
  private final double intakeAngleDeployed = Constants.Intake.kIntakeAngleDeployed;
  private final double intakeAngleStowed = Constants.Intake.kIntakeAngleStowed;

  //Pivot Constants
  private final DCMotor dcMotorPivot = DCMotor.getKrakenX60(1);
  private final int canIDPivot = Constants.Intake.kPivotCanID;
  private final double gearRatioPivot = Constants.Intake.kPivotGearRatio;
  private final double PivotkP = Constants.Intake.kPivotKP;
  private final double PivotkI = Constants.Intake.kPivotKI;
  private final double PivotkD = Constants.Intake.kPivotKD;
  private final double PivotkS = Constants.Intake.kPivotKS;
  private final double PivotkV = Constants.Intake.kPivotKV;
  private final double PivotkA = Constants.Intake.kPivotKA;
  private final double PivotkG = Constants.Intake.kPivotKG;
  private final double PivotMaxVelocity = Constants.Intake.kPivotMaxVelocity;
  private final double PivotMaxAcceleration = Constants.Intake.kPivotMaxAcceleration;
  private final boolean IsPivotBrakeEnabled = Constants.Intake.kPivotBrakeEnabled;
  private final double PivotForwardSoftLimit = Constants.Intake.kPivotForwardSoftLimit;
  private final double PivotReverseSoftLimit = Constants.Intake.kPivotReverseSoftLimit;
  private final boolean IsPivotStatorLimitEnabled = Constants.Intake.kPivotStatorLimitEnabled;
  private final double PivotStatorCurrentLimit = Constants.Intake.kPivotStatorCurrentLimit;
  private final boolean IsPivotSupplyLimitEnabled = Constants.Intake.kPivotSupplyLimitEnabled;
  private final double PivotSupplyCurrentLimit = Constants.Intake.kPivotSupplyCurrentLimit;

  // Constants for spinner
  private final DCMotor spinnerDcMotor = DCMotor.getKrakenX60(1);
  private final int spinnerCanID = Constants.Intake.kSpinnerCanID;
  private final double spinnerGearRatio = Constants.Intake.kSpinnerGearRatio;
  private final double spinnerKP = Constants.Intake.kSpinnerKP;
  private final double spinnerKI = Constants.Intake.kSpinnerKI;
  private final double spinnerKD = Constants.Intake.kSpinnerKD;
  private final double spinnerKS = Constants.Intake.kSpinnerKS;
  private final double spinnerKV = Constants.Intake.kSpinnerKV;
  private final double spinnerKA = Constants.Intake.kSpinnerKA;
  private final double spinnerKG = Constants.Intake.kSpinnerKG;
  private final double spinnerMaxVelocity = Constants.Intake.kSpinnerMaxVelocity;
  private final double spinnerMaxAcceleration = Constants.Intake.kSpinnerMaxAcceleration;
  private final boolean spinnerBrakeMode = Constants.Intake.kSpinnerBrakeMode;
  private final double spinnerForwardSoftLimit = Constants.Intake.kSpinnerForwardSoftLimit;
  private final double spinnerReverseSoftLimit = Constants.Intake.kSpinnerReverseSoftLimit;
  private final boolean spinnerEnableStatorLimit = Constants.Intake.kSpinnerEnableStatorLimit;
  private final int spinnerStatorCurrentLimit = Constants.Intake.kSpinnerStatorCurrentLimit;
  private final boolean spinnerEnableSupplyLimit = Constants.Intake.kSpinnerEnableSupplyLimit;
  private final double spinnerSupplyCurrentLimit = Constants.Intake.kSpinnerSupplyCurrentLimit;

  // Feedforward
  private final ArmFeedforward Pivotfeedforward = new ArmFeedforward(
    PivotkS, // kS
    PivotkG, // kG - gravity compensation for pivot
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
  private final TalonFX SpinnerMotor;
  private final VelocityVoltage spinnerVelocityRequest;
  private final StatusSignal<Angle> spinnerPositionSignal;
  private final StatusSignal<AngularVelocity> spinnerVelocitySignal;
  private final StatusSignal<Voltage> spinnerVoltageSignal;
  private final StatusSignal<Current> spinnerStatorCurrentSignal;
  private final StatusSignal<Temperature> spinnerTemperatureSignal;
  

  // Simulation
  private final SingleJointedArmSim intakeSim;
  private final FlywheelSim spinnerSim;

  /**
   * Creates a new intake Subsystem.
   */
  public Intake() {
    // Initialize pivot motor controller
    PivotMotor = new TalonFX(canIDPivot);

    // Initialize spinner motor controller
    SpinnerMotor = new TalonFX(spinnerCanID);

    //pivot Create control requests
    pivotPositionRequest = new PositionVoltage(0).withSlot(0);
    pivotVelocityRequest = new VelocityVoltage(0).withSlot(0);

    // Spinner create control requests
    spinnerVelocityRequest = new VelocityVoltage(0).withSlot(0);

    // pivot get status signals
    pivotPositionSignal = PivotMotor.getPosition();
    pivotVelocitySignal = PivotMotor.getVelocity();
    pivotVoltageSignal = PivotMotor.getMotorVoltage();
    pivotStatorCurrentSignal = PivotMotor.getStatorCurrent();
    pivotTemperatureSignal = PivotMotor.getDeviceTemp();

    // Spinner get status signals
    spinnerPositionSignal = SpinnerMotor.getPosition();
    spinnerVelocitySignal = SpinnerMotor.getVelocity();
    spinnerVoltageSignal = SpinnerMotor.getMotorVoltage();
    spinnerStatorCurrentSignal = SpinnerMotor.getStatorCurrent();
    spinnerTemperatureSignal = SpinnerMotor.getDeviceTemp();

    TalonFXConfiguration PivotMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration SpinnerMotorConfig = new TalonFXConfiguration();

    //Pivot Configure PID for slot 0
    Slot0Configs slot0 = PivotMotorConfig.Slot0;
    slot0.kP = PivotkP;
    slot0.kI = PivotkI;
    slot0.kD = PivotkD;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kS = PivotkS;
    slot0.kV = PivotkV;
    slot0.kA = PivotkA;

    // Spinner Configure PID for slot 0
    Slot0Configs spinnerSlot0 = SpinnerMotorConfig.Slot0;
    spinnerSlot0.kP = spinnerKP;
    spinnerSlot0.kI = spinnerKI;
    spinnerSlot0.kD = spinnerKD;
    spinnerSlot0.kS = spinnerKS;
    spinnerSlot0.kV = spinnerKV;
    spinnerSlot0.kA = spinnerKA;

    // Pivot Set current limits
    CurrentLimitsConfigs currentLimits = PivotMotorConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = PivotStatorCurrentLimit;
    currentLimits.StatorCurrentLimitEnable = IsPivotStatorLimitEnabled;
    currentLimits.SupplyCurrentLimit = PivotSupplyCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = IsPivotSupplyLimitEnabled;

    // Set spinner current limits
    CurrentLimitsConfigs spinnerCurrentLimits = SpinnerMotorConfig.CurrentLimits;
    spinnerCurrentLimits.StatorCurrentLimit = spinnerStatorCurrentLimit;
    spinnerCurrentLimits.StatorCurrentLimitEnable = spinnerEnableStatorLimit;
    spinnerCurrentLimits.SupplyCurrentLimit = spinnerSupplyCurrentLimit;
    spinnerCurrentLimits.SupplyCurrentLimitEnable = spinnerEnableSupplyLimit;

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

    // Spinner set brake mode
    SpinnerMotorConfig.MotorOutput.NeutralMode = spinnerBrakeMode
      ? NeutralModeValue.Brake
      : NeutralModeValue.Coast;

    // pivot Apply gear ratio
    PivotMotorConfig.Feedback.SensorToMechanismRatio = gearRatioPivot;

    // Spinner apply gear ratio
    SpinnerMotorConfig.Feedback.SensorToMechanismRatio = spinnerGearRatio;

    // pivot Apply configuration
    PivotMotor.getConfigurator().apply(PivotMotorConfig);

    // Spinner apply configuration
    SpinnerMotor.getConfigurator().apply(SpinnerMotorConfig);

    // pivot Reset encoder position
    PivotMotor.setPosition(0);

    // Spinner reset encoder position
    SpinnerMotor.setPosition(0);

    // Initialize simulation
    intakeSim = new SingleJointedArmSim(
      dcMotorPivot, // Motor type
      gearRatioPivot,
      Constants.Intake.kSimArmMomentOfInertia,
      Constants.Intake.kSimArmLength,
      Units.degreesToRadians(Constants.Intake.kSimMinAngleDegrees),
      Units.degreesToRadians(Constants.Intake.kSimMaxAngleDegrees),
      Constants.Intake.kSimulateGravity,
      Units.degreesToRadians(Constants.Intake.kSimStartingPositionDegrees)
    );

    // Initialize spinner simulation
    LinearSystem<N1, N1, N1> spinnerPlant = LinearSystemId.createFlywheelSystem(
      DCMotor.getKrakenX60(1),
      Constants.Intake.kSimSpinnerMomentOfInertia,
      spinnerGearRatio
    );
    spinnerSim = new FlywheelSim(spinnerPlant, DCMotor.getKrakenX60(1));
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
      pivotTemperatureSignal,
      spinnerPositionSignal,
      spinnerVelocitySignal,
      spinnerVoltageSignal,
      spinnerStatorCurrentSignal,
      spinnerTemperatureSignal
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

    // Simulate spinner motor
    spinnerSim.setInput(SpinnerMotor.getSimState().getMotorVoltage());
    spinnerSim.update(0.020);

    // Update spinner motor sim state
    double spinnerVelocityRadPerSec = spinnerSim.getAngularVelocityRadPerSec();
    double spinnerMotorVelocity = RadiansPerSecond.of(
      spinnerVelocityRadPerSec * spinnerGearRatio
    ).in(RotationsPerSecond);

    SpinnerMotor.getSimState().setRotorVelocity(spinnerMotorVelocity);
    
    // Update battery voltage considering both motors
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        intakeSim.getCurrentDrawAmps() + spinnerSim.getCurrentDrawAmps()
      )
    );
  }


  //------------------------ Pivot Methods -----------------------//
  /**
   * Get the current position in Rotations.
   * @return Position in Rotations
   */
  @Logged(name = "Pivot Position/Rotations")
  public double   GetPivotPosition() {
    // Rotations
    return pivotPositionSignal.getValueAsDouble();
  }

  /**
   * Get the current position in Degrees.
   * @return Position in Degrees.
   */
  @Logged(name = "Pivot Position/Degrees")
  public double GetPivotPositionDegrees() {
    return GetPivotPosition() * 360;
  }

  /**
   * Get the current velocity in rotations per second.
   * @return Velocity in rotations per second
   */
  @Logged(name = "Pivot Velocity/RotationsPerSecond")
  public double GetPivotVelocity() {
    return pivotVelocitySignal.getValueAsDouble();
  }

  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
  @Logged(name = "Pivot Voltage")
  public double GetPivotVoltage() {
    return pivotVoltageSignal.getValueAsDouble();
  }

  /**
   * Get the current motor current.
   * @return Motor current in amps
   */
  public double GetPivotCurrent() {
    return pivotStatorCurrentSignal.getValueAsDouble();
  }

  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
   */
  public double GetPivotTemperature() {
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

    double PivotffVolts = Pivotfeedforward.calculate(GetPivotVelocity(), acceleration);
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

    double ffVolts = Pivotfeedforward.calculate(GetPivotVelocity(), acceleration);
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
    return runOnce(() -> PivotMotor.stopMotor());
  }

  public Command PivotSetAngleCommand(double angleDegrees) {
    return run(() -> PivotSetAngle(angleDegrees));
  }

  //------------------------ Spinner Methods -----------------------//
  /**

  /**
   * Get the current velocity in rotations per second.
   * @return Velocity in rotations per second
   */
  @Logged(name = "Spinner Vel")
  public double GetSpinnerVelocity() {
    return spinnerVelocitySignal.getValueAsDouble();
  }

  @Logged(name = "Target Spinner Vel")
  public double GetSpinnerTargetVelocity() {
    return SpinnerMotor.getClosedLoopReference().getValueAsDouble();
  }

  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
  @Logged(name = "Spinner Voltage")
  public double GetSpinnerVoltage() {
    return spinnerVoltageSignal.getValueAsDouble();
  }


  /**
   * Get the current motor current.
   * @return Motor current in amps
   */
  public double GetSpinnerCurrent() {
    return spinnerStatorCurrentSignal.getValueAsDouble();
  }

  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
   */
  public double GetSpinnerTemperature() {
    return spinnerTemperatureSignal.getValueAsDouble();
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
    double ffVolts = spinnerFeedforward.calculate(GetSpinnerVelocity(), acceleration);
    SpinnerMotor.setControl(spinnerVelocityRequest.withVelocity(velocityRotSec));
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
    return runOnce(() -> {
      // Update velocity request to 0 so logged target shows 0
      SpinnerMotor.setControl(spinnerVelocityRequest.withVelocity(0));
      // Then immediately stop motor (neutral output)
      SpinnerMotor.stopMotor();
    });
  }

  /**
   * Creates a command to move the spinner at a specific velocity.
   * @param velocityRotSec The target velocity in rotations per second
   * @return A command that moves the spinner at the specified velocity
   */
  public Command SpinnerMoveAtVelocityCommand(double velocityRotSec) {
    return run(() -> SpinnerSetVelocity(velocityRotSec));
  }

  //------------------------ Tuning -----------------------//

  /**
   * Sets spinner velocity using tunable PID values and setpoint from dashboard.
   * Updates PID gains in real-time and uses Setpoint1 for target velocity.
   */
  private void SpinnerTunable(DashboardPublisher dashboard) {
    if (dashboard == null) {
      return; // Skip if no dashboard publisher
    }

    // Get tunable PID values
    double kP = dashboard.getTunableKP();
    double kI = dashboard.getTunableKI();
    double kD = dashboard.getTunableKD();
    double kV = dashboard.getTunableKV();
    double kA = dashboard.getTunableKA();
    double setpoint = dashboard.getTunableSetpoint1();

    // Update PID gains in slot 0
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kV = kV;
    slot0.kA = kA;
    slot0.kS = spinnerKS; // Keep original kS
    
    SpinnerMotor.getConfigurator().apply(slot0);

    // Set velocity using tunable setpoint
    SpinnerSetVelocity(setpoint);
  }

  /**
   * Command to tune spinner using dashboard values.
   * Continuously updates PID and setpoint from dashboard.
   */
  public Command SpinnerTunableCommand(DashboardPublisher dashboard) {
    return run(() -> SpinnerTunable(dashboard));
  }


  private void pivotTunable(DashboardPublisher dashboard) {
    if (dashboard == null) {
      return; // Skip if no dashboard publisher
    }

    // Get tunable PID values
    double kP = dashboard.getTunableKP();
    double kI = dashboard.getTunableKI();
    double kD = dashboard.getTunableKD();
    double kV = dashboard.getTunableKV();
    double kA = dashboard.getTunableKA();
    double setpoint = dashboard.getTunableSetpoint1();

    // Update PID gains in slot 0
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kV = kV;
    slot0.kA = kA;
    slot0.kS = PivotkS; // Keep original kS
    
    PivotMotor.getConfigurator().apply(slot0);

    // Set position using tunable setpoint
    PivotSetAngle(setpoint);
  }

  /**
   * Command to tune pivot using dashboard values.
   * Continuously updates PID and setpoint from dashboard.
   */
  public Command PivotTunableCommand(DashboardPublisher dashboard) {
    return run(() -> pivotTunable(dashboard));
  }

  //------------------------ Useful Commands -----------------------//

  private void stow(){
    PivotSetAngle(intakeAngleStowed);
    SpinnerStopCommand().schedule();
  }

  public Command StowCommand() {
    return runOnce(() -> stow());
  }

  private void deploy(){
    PivotSetAngle(intakeAngleDeployed);
    SpinnerSetVelocity(spinnerSpeed);
  }
  public Command DeployCommand() {
    return runOnce(() -> deploy());
  }

  
}