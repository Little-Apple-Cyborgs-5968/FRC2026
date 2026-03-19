package frc.robot.subsystems.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber;

/**
 * Visualization for the climber subsystem in simulation.
 * Shows a vertical elevator-like mechanism.
 */
public class ClimberSim extends SubsystemBase {

  private final Climber climber;

  // Simulation display
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d baseFrame;
  private final MechanismLigament2d carriage;

  /**
   * Creates a new visualization for the climber.
   *
   * @param climberSubsystem The climber subsystem to visualize
   */
  public ClimberSim(Climber climberSubsystem) {
    this.climber = climberSubsystem;

    // Create the simulation display
    mech = new Mechanism2d(1.0, 1.5);

    // Climber root at (0.5, 0.1)
    root = mech.getRoot("climberRoot", 0.5, 0.1);

    // Frame (acts as the track)
    baseFrame = root.append(
      new MechanismLigament2d(
        "ClimberFrame",
        1.2, // Length representing max height
        90,  // Vertical
        10,  // Line width
        new Color8Bit(Color.kDarkGray)
      )
    );

    // Carriage (moves up and down)
    carriage = root.append(
      new MechanismLigament2d(
        "ClimberCarriage",
        0.1, // Small starting length, which we will adjust to represent carriage position
        90,  
        20,  // Wider than the frame
        new Color8Bit(Color.kRed)
      )
    );

    // Initialize visualization
    SmartDashboard.putData("CLIMBER/Climber Sim", mech);
  }

  @Override
  public void periodic() {
    // Get the simulated position in mechanism rotations
    double posRotations = climber.getPosition();

    // Map the position. The visual carriage length is set proportional to position to simulate it moving up.
    // Ensure height > 0 so the ligament doesn't vanish/error out, give it a tiny base length
    double minVisualLength = 0.05;
    // Scale the rotations to visually fit the 1.2 frame length
    // Assume 105 rotations is max length (1.2)
    double displayedLength = Math.max(minVisualLength, (posRotations / 105.0) * 1.2 + minVisualLength);

    carriage.setLength(displayedLength);

    // Add telemetry data
    SmartDashboard.putNumber("CLIMBER/Velocity (RPS)", climber.getVelocity());
    SmartDashboard.putNumber("CLIMBER/Position (Rotations)", climber.getPosition());
    SmartDashboard.putNumber("CLIMBER/Target Position (Rotations)", climber.getTargetPosition());
    SmartDashboard.putNumber("CLIMBER/Voltage (V)", climber.getVoltage());
    SmartDashboard.putNumber("CLIMBER/Current (A)", climber.getCurrent());
    SmartDashboard.putNumber("CLIMBER/Temperature (C)", climber.getTemperature());
    SmartDashboard.putNumber("CLIMBER/Sim Pos (Rotations)", posRotations);
    SmartDashboard.putNumber("CLIMBER/Sim Current Draw (A)", climber.getSimulation().getCurrentDrawAmps());
  }
}
