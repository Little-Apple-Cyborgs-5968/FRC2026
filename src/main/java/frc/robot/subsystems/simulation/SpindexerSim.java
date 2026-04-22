package frc.robot.subsystems.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Spindexer;

/**
 * Visualization for the spindexer subsystem in simulation.
 */
public class SpindexerSim extends SubsystemBase {

  private final Spindexer spindexer;

  // Simulation display
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d spindexerWheel;
  private final MechanismLigament2d spindexerSpoke1;
  private final MechanismLigament2d spindexerSpoke2;
  private final MechanismLigament2d spindexerSpoke3;
  private final MechanismLigament2d spindexerSpoke4;

  // Visualization constants
  private final double WHEEL_RADIUS = 0.15;
  private final double SPOKE_WIDTH = 4;
  private final double WHEEL_WIDTH = 8;

  // Animation tracking
  private double currentAngle = 0;

  /**
   * Creates a new visualization for the spindexer.
   *
   * @param spindexerSubsystem The spindexer subsystem to visualize
   */
  public SpindexerSim(Spindexer spindexerSubsystem) {
    this.spindexer = spindexerSubsystem;

    // Create the simulation display
    mech = new Mechanism2d(0.8, 0.8);
    root = mech.getRoot("spindexerRoot", 0.4, 0.4);

    // Add outer wheel (visual boundary)
    spindexerWheel = root.append(
      new MechanismLigament2d(
        "Wheel",
        WHEEL_RADIUS,
        0,
        WHEEL_WIDTH,
        new Color8Bit(Color.kDarkGreen)
      )
    );

    // Add spokes to visualize rotation
    spindexerSpoke1 = root.append(
      new MechanismLigament2d(
        "Spoke1",
        WHEEL_RADIUS,
        0,
        SPOKE_WIDTH,
        new Color8Bit(Color.kYellow)
      )
    );

    spindexerSpoke2 = root.append(
      new MechanismLigament2d(
        "Spoke2",
        WHEEL_RADIUS,
        90,
        SPOKE_WIDTH,
        new Color8Bit(Color.kYellow)
      )
    );

    spindexerSpoke3 = root.append(
      new MechanismLigament2d(
        "Spoke3",
        WHEEL_RADIUS,
        180,
        SPOKE_WIDTH,
        new Color8Bit(Color.kYellow)
      )
    );

    spindexerSpoke4 = root.append(
      new MechanismLigament2d(
        "Spoke4",
        WHEEL_RADIUS,
        270,
        SPOKE_WIDTH,
        new Color8Bit(Color.kYellow)
      )
    );

    // Initialize visualization
    SmartDashboard.putData("SPINDEXER/Spindexer Sim", mech);
  }

  @Override
  public void periodic() {
    // Update rotation angle based on velocity
    double velocity = spindexer.getVelocity(); // rotations per second
    double deltaAngle = velocity * 360.0 * 0.02; // degrees per 20ms period
    currentAngle += deltaAngle;
    
    // Keep angle in [0, 360) range
    currentAngle = currentAngle % 360;

    // Update spoke angles to show rotation
    spindexerWheel.setAngle(currentAngle);
    spindexerSpoke1.setAngle(currentAngle);
    spindexerSpoke2.setAngle(currentAngle + 90);
    spindexerSpoke3.setAngle(currentAngle + 180);
    spindexerSpoke4.setAngle(currentAngle + 270);

    // Color coding based on velocity
    Color8Bit spokeColor;
    if (Math.abs(velocity) < 0.1) {
      spokeColor = new Color8Bit(Color.kGray); // Stopped
    } else if (velocity > 0) {
      spokeColor = new Color8Bit(Color.kGreen); // Forward
    } else {
      spokeColor = new Color8Bit(Color.kRed); // Reverse
    }

    spindexerSpoke1.setColor(spokeColor);
    spindexerSpoke2.setColor(spokeColor);
    spindexerSpoke3.setColor(spokeColor);
    spindexerSpoke4.setColor(spokeColor);

    // Add telemetry data
    SmartDashboard.putNumber("SPINDEXER/Velocity (RPS)", velocity);
    SmartDashboard.putNumber("SPINDEXER/Target Velocity (RPS)", spindexer.getTargetVelocity());
    SmartDashboard.putNumber("SPINDEXER/Voltage (V)", spindexer.getVoltage());
    SmartDashboard.putNumber("SPINDEXER/Stator Current (A)", spindexer.getStatorCurrent());
    SmartDashboard.putNumber("SPINDEXER/Temperature (C)", spindexer.getTemperature());
    SmartDashboard.putNumber(
      "SPINDEXER/Sim Current Draw (A)",
      spindexer.getSimulation().getCurrentDrawAmps()
    );
  }
}
