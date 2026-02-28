package frc.robot.subsystems.simulation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter;

/**
 * Telemetry for the shooter subsystem in simulation.
 */
public class ShooterSim extends SubsystemBase {

  private final Shooter shooter;

  /**
   * Creates a new ShooterSim.
   */
  public ShooterSim(Shooter shooterSubsystem) {
    this.shooter = shooterSubsystem;
  }

  @Override
  public void periodic() {
    double hoodAngle = shooter.getHoodAngle();
    double leftVelocity = shooter.getLeftFlywheelVelocity();
    double rightVelocity = shooter.getRightFlywheelVelocity();

    // Add telemetry data
    SmartDashboard.putNumber("SHOOTER/Left Flywheel RPS", leftVelocity);
    SmartDashboard.putNumber("SHOOTER/Right Flywheel RPS", rightVelocity);
    SmartDashboard.putNumber("SHOOTER/Average Flywheel RPS", shooter.getAverageFlywheelVelocity());
    SmartDashboard.putNumber("SHOOTER/Target Flywheel RPS", shooter.getTargetFlywheelVelocity());
    SmartDashboard.putBoolean("SHOOTER/At Speed", shooter.isFlywheelAtSpeed());
    SmartDashboard.putNumber("SHOOTER/Flywheel Voltage", shooter.getFlywheelVoltage());
    SmartDashboard.putNumber("SHOOTER/Flywheel Current", shooter.getFlywheelCurrent());
    SmartDashboard.putNumber("SHOOTER/Hood Angle (deg)", hoodAngle);
    SmartDashboard.putNumber("SHOOTER/Hood Target Angle (deg)", shooter.getTargetHoodAngle());
    SmartDashboard.putNumber("SHOOTER/Hood Velocity (deg/s)", shooter.getHoodVelocity());
    SmartDashboard.putBoolean("SHOOTER/Hood At Target", shooter.isHoodAtTarget());
    SmartDashboard.putNumber(
      "SHOOTER/Sim Current Draw (A)",
      shooter.getLeftFlywheelSim().getCurrentDrawAmps() + shooter.getHoodSim().getCurrentDrawAmps()
    );
  }
}
