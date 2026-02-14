package frc.robot.subsystems.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter;

/**
 * Visualization for the shooter subsystem in simulation.
 * Shows dual counter-rotating flywheels and adjustable hood.
 */
public class ShooterSim extends SubsystemBase {

  private final Shooter shooter;

  // Simulation display
  private final Mechanism2d mech;
  
  // Hood
  private final MechanismRoot2d hoodRoot;
  private final MechanismLigament2d hoodArm;
  
  // Flywheels
  private final MechanismRoot2d leftFlywheelRoot;
  private final MechanismRoot2d rightFlywheelRoot;
  private final MechanismLigament2d leftFlywheel;
  private final MechanismLigament2d rightFlywheel;
  private final MechanismLigament2d leftSpoke1;
  private final MechanismLigament2d leftSpoke2;
  private final MechanismLigament2d rightSpoke1;
  private final MechanismLigament2d rightSpoke2;

  // Visualization constants
  private final double HOOD_LENGTH = 0.3;
  private final double HOOD_WIDTH = 8;
  private final double FLYWHEEL_RADIUS = 0.1;
  private final double FLYWHEEL_WIDTH = 10;
  private final double SPOKE_WIDTH = 4;

  // Animation tracking
  private double leftFlywheelAngle = 0;
  private double rightFlywheelAngle = 0;

  /**
   * Creates a new visualization for the shooter.
   */
  public ShooterSim(Shooter shooterSubsystem) {
    this.shooter = shooterSubsystem;

    // Create the simulation display
    mech = new Mechanism2d(1.0, 1.0);
    
    // Hood mechanism (starts at 90 degrees = vertical)
    hoodRoot = mech.getRoot("hoodRoot", 0.5, 0.2);
    hoodArm = hoodRoot.append(
      new MechanismLigament2d(
        "Hood",
        HOOD_LENGTH,
        90, // Start vertical
        HOOD_WIDTH,
        new Color8Bit(Color.kGray)
      )
    );

    // Left flywheel (counter-clockwise rotation)
    leftFlywheelRoot = mech.getRoot("leftFlywheelRoot", 0.35, 0.5);
    leftFlywheel = leftFlywheelRoot.append(
      new MechanismLigament2d(
        "LeftFlywheel",
        FLYWHEEL_RADIUS,
        0,
        FLYWHEEL_WIDTH,
        new Color8Bit(Color.kDarkRed)
      )
    );
    leftSpoke1 = leftFlywheelRoot.append(
      new MechanismLigament2d(
        "LeftSpoke1",
        FLYWHEEL_RADIUS,
        0,
        SPOKE_WIDTH,
        new Color8Bit(Color.kRed)
      )
    );
    leftSpoke2 = leftFlywheelRoot.append(
      new MechanismLigament2d(
        "LeftSpoke2",
        FLYWHEEL_RADIUS,
        180,
        SPOKE_WIDTH,
        new Color8Bit(Color.kRed)
      )
    );

    // Right flywheel (clockwise rotation - opposite)
    rightFlywheelRoot = mech.getRoot("rightFlywheelRoot", 0.65, 0.5);
    rightFlywheel = rightFlywheelRoot.append(
      new MechanismLigament2d(
        "RightFlywheel",
        FLYWHEEL_RADIUS,
        0,
        FLYWHEEL_WIDTH,
        new Color8Bit(Color.kDarkRed)
      )
    );
    rightSpoke1 = rightFlywheelRoot.append(
      new MechanismLigament2d(
        "RightSpoke1",
        FLYWHEEL_RADIUS,
        0,
        SPOKE_WIDTH,
        new Color8Bit(Color.kRed)
      )
    );
    rightSpoke2 = rightFlywheelRoot.append(
      new MechanismLigament2d(
        "RightSpoke2",
        FLYWHEEL_RADIUS,
        180,
        SPOKE_WIDTH,
        new Color8Bit(Color.kRed)
      )
    );

    // Initialize visualization
    SmartDashboard.putData("SHOOTER/Shooter Sim", mech);
  }

  @Override
  public void periodic() {
    // Update hood angle
    double hoodAngle = shooter.getHoodAngle();
    hoodArm.setAngle(hoodAngle);

    // Color hood based on position
    if (shooter.isHoodAtTarget()) {
      hoodArm.setColor(new Color8Bit(Color.kGreen));
    } else {
      hoodArm.setColor(new Color8Bit(Color.kYellow));
    }

    // Update flywheel animations
    double leftVelocity = shooter.getLeftFlywheelVelocity(); // RPS
    double rightVelocity = shooter.getRightFlywheelVelocity(); // RPS (will be negative)
    
    double leftDeltaAngle = leftVelocity * 360.0 * 0.02; // degrees per 20ms
    double rightDeltaAngle = rightVelocity * 360.0 * 0.02;
    
    leftFlywheelAngle += leftDeltaAngle;
    rightFlywheelAngle += rightDeltaAngle;
    
    leftFlywheelAngle = leftFlywheelAngle % 360;
    rightFlywheelAngle = rightFlywheelAngle % 360;

    // Update left flywheel visualization
    leftFlywheel.setAngle(leftFlywheelAngle);
    leftSpoke1.setAngle(leftFlywheelAngle);
    leftSpoke2.setAngle(leftFlywheelAngle + 180);

    // Update right flywheel visualization
    rightFlywheel.setAngle(rightFlywheelAngle);
    rightSpoke1.setAngle(rightFlywheelAngle);
    rightSpoke2.setAngle(rightFlywheelAngle + 180);

    // Color flywheels based on speed
    Color8Bit flywheelColor;
    if (shooter.isFlywheelAtSpeed()) {
      flywheelColor = new Color8Bit(Color.kGreen); // At speed
    } else if (Math.abs(leftVelocity) > 1.0) {
      flywheelColor = new Color8Bit(Color.kYellow); // Spinning up
    } else {
      flywheelColor = new Color8Bit(Color.kGray); // Stopped
    }

    leftSpoke1.setColor(flywheelColor);
    leftSpoke2.setColor(flywheelColor);
    rightSpoke1.setColor(flywheelColor);
    rightSpoke2.setColor(flywheelColor);

    // Add telemetry data
    SmartDashboard.putNumber("SHOOTER/Left Flywheel RPS", leftVelocity);
    SmartDashboard.putNumber("SHOOTER/Right Flywheel RPS", rightVelocity);
    SmartDashboard.putNumber("SHOOTER/Average Flywheel RPS", shooter.getAverageFlywheelVelocity());
    SmartDashboard.putNumber("SHOOTER/Target Flywheel RPS", shooter.getTargetFlywheelVelocity());
    SmartDashboard.putBoolean("SHOOTER/At Speed", shooter.isFlywheelAtSpeed());
    SmartDashboard.putNumber("SHOOTER/Flywheel Voltage", shooter.getFlywheelVoltage());
    SmartDashboard.putNumber("SHOOTER/Flywheel Current", shooter.getFlywheelCurrent());
    SmartDashboard.putNumber("SHOOTER/Hood Angle (deg)", hoodAngle);
    SmartDashboard.putNumber("SHOOTER/Hood Velocity (deg/s)", shooter.getHoodVelocity());
    SmartDashboard.putBoolean("SHOOTER/Hood At Target", shooter.isHoodAtTarget());
    SmartDashboard.putNumber(
      "SHOOTER/Sim Current Draw (A)",
      shooter.getLeftFlywheelSim().getCurrentDrawAmps() + shooter.getHoodSim().getCurrentDrawAmps()
    );
  }
}
