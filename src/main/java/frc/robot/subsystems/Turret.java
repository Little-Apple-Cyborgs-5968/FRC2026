package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.driverIO.DashboardPublisher;
import frc.robot.utils.TurretUtil;
import java.util.function.Supplier;

/**
 * Pivot subsystem using TalonFX with Krakenx60 motor
 */
@Logged(name = "Turret")
public class Turret extends SubsystemBase {

  // Constants
  private final DCMotor dcMotor = DCMotor.getKrakenX60(1);
  private final int canID = Constants.Turret.kTurretCanID;
  private final double gearRatio = Constants.Turret.kGearRatio;
  private final double kP = Constants.Turret.kKP;
  private final double kI = Constants.Turret.kKI;
  private final double kD = Constants.Turret.kKD;
  private final double kS = Constants.Turret.kKS;
  private final double kV = Constants.Turret.kKV;
  private final double kA = Constants.Turret.kKA;
  private final double kG = Constants.Turret.kKG;
  private final double maxVelocity = Constants.Turret.kMaxVelocity;
  private final double maxAcceleration = Constants.Turret.kMaxAcceleration;
  private final boolean brakeMode = Constants.Turret.kBrakeMode;
  private final boolean enableStatorLimit = Constants.Turret.kEnableStatorLimit;
  private final double statorCurrentLimit = Constants.Turret.kStatorCurrentLimit;
  private final boolean enableSupplyLimit = Constants.Turret.kEnableSupplyLimit;
  private final double supplyCurrentLimit = Constants.Turret.kSupplyCurrentLimit;

  // Feedforward
  private final ArmFeedforward feedforward = new ArmFeedforward(
    kS, // kS
    0, // kG - Pivot doesn't need gravity compensation
    kV, // kV
    kA // kA
  );

  // Motor controller
  private final TalonFX motor;
  private final MotionMagicVoltage positionRequest;
  private final VelocityVoltage velocityRequest;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  // Target tracking for telemetry
  private double targetAngleDegrees = 0.0;
  private double targetVelocityDegPerSec = 0.0;

  // Pose supplier — injected by RobotContainer to enable distance-to-target telemetry
  private Supplier<Pose2d> robotPoseSupplier = null;
  // Cached distance to hub, updated every periodic()
  private double distanceToTargetMeters = 0.0;

  // Simulation
  private final SingleJointedArmSim pivotSim;

  /**
   * Creates a new Pivot Subsystem.
   */
  public Turret() {
    // Initialize motor controller
    motor = new TalonFX(canID);

    // Create control requests
    positionRequest = new MotionMagicVoltage(0).withSlot(0);
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    // get status signals
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Configure PID for slot 0
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;

    // Configure Motion Magic velocity/acceleration limits
    MotionMagicConfigs motionMagic = config.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = maxVelocity;     // rotations/s (mechanism-side)
    motionMagic.MotionMagicAcceleration   = maxAcceleration; // rotations/s² (mechanism-side)

    // Set current limits
    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimit = statorCurrentLimit;
    currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

    // Set brake mode
    config.MotorOutput.NeutralMode = brakeMode
      ? NeutralModeValue.Brake
      : NeutralModeValue.Coast;

    // Apply gear ratio
    config.Feedback.SensorToMechanismRatio = gearRatio;

    // Apply configuration
    motor.getConfigurator().apply(config);

    // Set encoder to the physical starting angle
    double startAngleDegrees = Constants.Turret.kTurretStartAngleDegrees;
    double startMechanismRotations = startAngleDegrees / 360.0;
    // motor.setPosition takes rotor rotations; SensorToMechanismRatio is applied to reads only
    motor.setPosition(startMechanismRotations * gearRatio);

    // Initialize simulation — start at the same physical angle so sim matches reality
    pivotSim = new SingleJointedArmSim(
      dcMotor, // Motor type
      gearRatio,
      0.01, // Arm moment of inertia - Small value since there are no arm parameters
      0.1, // Arm length (m) - Small value since there are no arm parameters
      Units.degreesToRadians(-180), // Min angle (rad)
      Units.degreesToRadians(180), // Max angle (rad)
      false, // Simulate gravity - Disable gravity for pivot
      Units.degreesToRadians(startAngleDegrees) // Starting position matches physical start angle
    );
  }

  /**
   * Update simulation and telemetry.
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

    // Compute and publish distance to hub whenever a pose supplier is registered
    if (robotPoseSupplier != null) {
      Pose2d pose = robotPoseSupplier.get();
      if (pose != null) {
        distanceToTargetMeters = TurretUtil.getDistance(pose, TurretUtil.TargetType.HUB);
        SmartDashboard.putNumber("DASHBOARD/Turret Distance To Hub (m)", distanceToTargetMeters);
      }
    }
  }

  /**
   * Update simulation.
   */
  @Override
  public void simulationPeriodic() {
    // Set input voltage from motor controller to simulation
    // Note: This may need to be talonfx.getSimState().getMotorVoltage() as the input
    //pivotSim.setInput(dcMotor.getVoltage(dcMotor.getTorque(pivotSim.getCurrentDrawAmps()), pivotSim.getVelocityRadPerSec()));
    // pivotSim.setInput(getVoltage());
    // Set input voltage from motor controller to simulation
    // Use motor voltage for TalonFX simulation input
    pivotSim.setInput(motor.getSimState().getMotorVoltage());

    // Update simulation by 20ms
    pivotSim.update(0.020);
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        pivotSim.getCurrentDrawAmps()
      )
    );

    double motorPosition = Radians.of(pivotSim.getAngleRads() * gearRatio).in(
      Rotations
    );
    double motorVelocity = RadiansPerSecond.of(
      pivotSim.getVelocityRadPerSec() * gearRatio
    ).in(RotationsPerSecond);

    motor.getSimState().setRawRotorPosition(motorPosition);
    motor.getSimState().setRotorVelocity(motorVelocity);
  }

  /**
   * Get the current position in Rotations (mechanism-side).
   * @return Position in Rotations
   */
  @Logged(name = "Position/Rotations")
  public double getPosition() {
    return positionSignal.getValueAsDouble();
  }

  /**
   * Get the current angle in degrees (mechanism-side).
   * @return Angle in degrees
   */
  @Logged(name = "Position/Degrees")
  public double getAngleDegrees() {
    return Units.rotationsToDegrees(getPosition());
  }

  /**
   * Get the current velocity in rotations per second (mechanism-side).
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity")
  public double getVelocity() {
    return velocitySignal.getValueAsDouble();
  }

  /**
   * Get the current velocity in degrees per second (mechanism-side).
   * @return Velocity in degrees per second
   */
  @Logged(name = "Velocity/DegreesPerSec")
  public double getVelocityDegreesPerSec() {
    return Units.rotationsToDegrees(getVelocity());
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
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
   */
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }

  /**
   * Get the target angle in degrees.
   * @return Target angle in degrees
   */
  @Logged(name = "Target/AngleDegrees")
  public double getTargetAngleDegrees() {
    return targetAngleDegrees;
  }

  /**
   * Get the target velocity in degrees per second.
   * @return Target velocity in degrees per second
   */
  @Logged(name = "Target/VelocityDegPerSec")
  public double getTargetVelocityDegPerSec() {
    return targetVelocityDegPerSec;
  }

  /**
   * Get the position error in degrees (target - current).
   * @return Position error in degrees
   */
  @Logged(name = "Error/AngleDegrees")
  public double getErrorDegrees() {
    double currentAngleDegrees = Units.rotationsToDegrees(getPosition());
    return targetAngleDegrees - currentAngleDegrees;
  }

  /**
   * Distance from the turret to the hub, in meters.
   * Returns 0 until a pose supplier is registered via {@link #setPoseSupplier}.
   */
  @Logged(name = "Target/DistanceMeters")
  public double getDistanceToTargetMeters() {
    return distanceToTargetMeters;
  }

  /**
   * Registers the robot pose supplier so the turret can compute distance-to-target telemetry.
   * Call this once from RobotContainer after all subsystems are constructed.
   *
   * @param poseSupplier Supplier that returns the current robot field pose
   */
  public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
    this.robotPoseSupplier = poseSupplier;
  }

  /**
   * Set pivot angle.
   * @param angleDegrees The target angle in degrees
   */
  public void setAngle(double angleDegrees) {
    setAngle(angleDegrees, 0);
  }

  /**
   * Set pivot angle with acceleration.
   * @param angleDegrees The target angle in degrees
   * @param acceleration The acceleration in rad/s²
   */
  public void setAngle(double angleDegrees, double acceleration) {
    // Track target for telemetry
    this.targetAngleDegrees = angleDegrees;
    this.targetVelocityDegPerSec = 0; // Position control, not velocity control
    
    // Convert degrees to rotations
    double angleRadians = Units.degreesToRadians(angleDegrees);
    double positionRotations = angleRadians / (2.0 * Math.PI);

    double ffVolts = feedforward.calculate(getVelocity(), acceleration);
    //motor.setControl(positionRequest.withPosition(positionRotations).withFeedForward(ffVolts));
    motor.setControl(positionRequest.withPosition(positionRotations));
  }

  /**
   * Set pivot angular velocity.
   * @param velocityDegPerSec The target velocity in degrees per second
   */
  public void setVelocity(double velocityDegPerSec) {
    setVelocity(velocityDegPerSec, 0);
  }

  /**
   * Set pivot angular velocity with acceleration.
   * @param velocityDegPerSec The target velocity in degrees per second
   * @param acceleration The acceleration in degrees per second squared
   */
  public void setVelocity(double velocityDegPerSec, double acceleration) {
    // Track target for telemetry
    this.targetVelocityDegPerSec = velocityDegPerSec;
    // Keep target angle at current position when doing velocity control
    this.targetAngleDegrees = Units.rotationsToDegrees(getPosition());
    
    // Convert degrees/sec to rotations/sec
    double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
    double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);

    double ffVolts = feedforward.calculate(getVelocity(), acceleration);
    //motor.setControl(velocityRequest.withVelocity(velocityRotations).withFeedForward(ffVolts));
    motor.setControl(velocityRequest.withVelocity(velocityRotations));
  }

  /**
   * Set motor voltage directly.
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    // When using direct voltage control, track current position as target
    this.targetAngleDegrees = Units.rotationsToDegrees(getPosition());
    this.targetVelocityDegPerSec = 0;
    
    motor.setVoltage(voltage);
  }

  /**
   * Get the pivot simulation for testing.
   * @return The pivot simulation model
   */
  public SingleJointedArmSim getSimulation() {
    return pivotSim;
  }

  /**
   * Creates a command to set the pivot to a specific angle.
   * @param angleDegrees The target angle in degrees
   * @return A command that sets the pivot to the specified angle
   */
  public Command setAngleCommand(double angleDegrees) {
    return runOnce(() -> setAngle(angleDegrees));
  }

  /**
   * Creates a command to move the pivot to a specific angle with a profile.
   * @param angleDegrees The target angle in degrees
   * @return A command that moves the pivot to the specified angle
   */
  public Command moveToAngleCommand(double angleDegrees) {
    return run(() -> {
      double currentAngle = Units.rotationsToDegrees(getPosition());
      double error = angleDegrees - currentAngle;
      double velocityDegPerSec =
        Math.signum(error) *
        Math.min(Math.abs(error) * 2.0, Units.radiansToDegrees(maxVelocity));
      setVelocity(velocityDegPerSec);
    })
      .until(() -> {
        double currentAngle = Units.rotationsToDegrees(getPosition());
        return Math.abs(angleDegrees - currentAngle) < 2.0; // 2 degree tolerance
      })
      .finallyDo(interrupted -> setVelocity(0));
  }

  /**
   * Creates a command to stop the pivot.
   * @return A command that stops the pivot
   */
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }

  /**
   * Creates a command to move the pivot at a specific velocity.
   * @param velocityDegPerSec The target velocity in degrees per second
   * @return A command that moves the pivot at the specified velocity
   */
  public Command moveAtVelocityCommand(double velocityDegPerSec) {
    return run(() -> setVelocity(velocityDegPerSec));
  }

  /**
   * Creates a command to automatically aim the turret at a target.
   * @param robotPoseSupplier Supplier that provides the current robot pose
   * @param target The target to aim at (HUB, LEFT_PASS, or RIGHT_PASS)
   * @return A command that continuously aims the turret at the target
   */
  public Command autoAimCommandTurret(Supplier<Pose2d> robotPoseSupplier, TurretUtil.TargetType target) {
    return run(() -> {
      Pose2d robotPose = robotPoseSupplier.get();
      TurretUtil.ShotSolution solution = TurretUtil.computeShotSolution(robotPose, target);
      
      if (solution.isValid) {
        setAngle(solution.turretAngleDegrees);
      }
    }).withName("AutoAim-" + target.toString());
  }

  /**
   * Continuously aims the turret using the "shoot on the move" iterative lead algorithm.
   * Velocity is supplied as a field-relative {@link ChassisSpeeds} (e.g. from the drivetrain
   * state converted via {@code ChassisSpeeds.fromRobotRelativeSpeeds}).
   *
   * @param robotPoseSupplier    Supplier for the current robot field pose
   * @param chassisSpeedsSupplier Supplier for the current field-relative chassis speeds
   * @param target               Which target to aim at
   * @return A command that continuously updates the turret angle with motion compensation
   */
  public Command shootOnMoveCommandTurret(Supplier<Pose2d> robotPoseSupplier,
                                           Supplier<ChassisSpeeds> chassisSpeedsSupplier,
                                           TurretUtil.TargetType target) {
    return run(() -> {
      Pose2d robotPose = robotPoseSupplier.get();
      ChassisSpeeds speeds = chassisSpeedsSupplier.get();
      TurretUtil.ShotSolution solution = TurretUtil.computeLeadShotSolution(
          robotPose, speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, target);

      if (solution.isValid) {
        setAngle(solution.turretAngleDegrees);
      }
    }).withName("ShootOnMove-Turret-" + target.toString());
  }

  //------------------------ Tuning -----------------------//

  // Cached tunable PID values — used to detect changes so we don't spam CAN bus
  private double lastTunableKP = Double.NaN;
  private double lastTunableKI = Double.NaN;
  private double lastTunableKD = Double.NaN;
  private double lastTunableKV = Double.NaN;
  private double lastTunableKA = Double.NaN;

  /**
   * Sets turret angle and velocity using tunable PID values and setpoints from dashboard.
   * Only re-applies PID config when values change (avoids flooding the CAN bus).
   * Setpoint1 controls angle (degrees), Setpoint2 controls velocity (deg/s).
   * @param dashboard The dashboard publisher for retrieving tunable values
   */
  private void turretTunable(DashboardPublisher dashboard) {
    if (dashboard == null) {
      return;
    }

    double newKP = dashboard.getTunableKP();
    double newKI = dashboard.getTunableKI();
    double newKD = dashboard.getTunableKD();
    double newKV = dashboard.getTunableKV();
    double newKA = dashboard.getTunableKA();
    double angleSetpoint = dashboard.getTunableSetpoint1();    // degrees
    double velocitySetpoint = dashboard.getTunableSetpoint2(); // degrees per second

    // Only apply new PID config when values actually change to avoid CAN bus flooding
    if (newKP != lastTunableKP || newKI != lastTunableKI || newKD != lastTunableKD
        || newKV != lastTunableKV || newKA != lastTunableKA) {
      lastTunableKP = newKP;
      lastTunableKI = newKI;
      lastTunableKD = newKD;
      lastTunableKV = newKV;
      lastTunableKA = newKA;

      Slot0Configs slot0 = new Slot0Configs();
      slot0.kP = newKP;
      slot0.kI = newKI;
      slot0.kD = newKD;
      slot0.kV = newKV;
      slot0.kA = newKA;
      slot0.kS = kS;
      motor.getConfigurator().apply(slot0);
    }

    // Setpoint1 = target angle in degrees, Setpoint2 = target velocity in deg/s
    if (Math.abs(angleSetpoint) > 0.01) {
      setAngle(angleSetpoint); // accepts degrees, converts to rotations internally
    } else if (Math.abs(velocitySetpoint) > 0.01) {
      setVelocity(velocitySetpoint); // accepts deg/s, converts to rot/s internally
    } else {
      setVelocity(0);
    }
  }

  /**
   * Command to tune turret using dashboard values.
   * Continuously updates PID and setpoints from dashboard.
   * Setpoint1 controls angle (degrees), Setpoint2 controls velocity (deg/s).
   * @param dashboard The dashboard publisher for retrieving tunable values
   * @return A command that tunes the turret using dashboard values
   */
  public Command TurretTunableCommand(DashboardPublisher dashboard) {
    return run(() -> turretTunable(dashboard));
  }

  /**
   * Rezero the turret encoder to 0 degrees.
   */
  private void rezero() {
    double rezeroMechanismRotations = Constants.Turret.kTurretRezeroAngleDegrees / 360.0;
    // motor.setPosition takes rotor rotations
    motor.setPosition(rezeroMechanismRotations * gearRatio);
  }

  /**
   * Command to rezero the turret encoder.
   */
  public Command RezeroCommand() {
    return runOnce(() -> rezero());
  }

}
