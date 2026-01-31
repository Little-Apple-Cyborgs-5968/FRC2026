package frc.robot.subsystems.simulation;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake;

/**
 * Visualization for the intake subsystem in simulation.
 */
public class IntakeSim extends SubsystemBase {

  private final Intake intake;

  // Simulation display
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d intakeMech;

  // Visualization constants
  private final double BASE_WIDTH = 0;
  private final double BASE_HEIGHT = 0;
  private final double intake_LENGTH = 0.3;
  private final double intake_WIDTH = 5;

  /**
   * Creates a new visualization for the intake.
   *
   * @param intakeSubsystem The intake subsystem to visualize
   */
  public IntakeSim(Intake intakeSubsystem) {
    this.intake = intakeSubsystem;

    // Create the simulation display
    mech = new Mechanism2d(1, 1);
    root = mech.getRoot("intakeRoot", 0.1, 0.2);

    // Add base
    MechanismLigament2d base = root.append(
      new MechanismLigament2d(
        "Base",
        BASE_WIDTH,
        0,
        BASE_HEIGHT,
        new Color8Bit(Color.kDarkGray)
      )
    );

    // Add intake point
    MechanismLigament2d intakePoint = base.append(
      new MechanismLigament2d(
        "intakePoint",
        0,
        90,
        0,
        new Color8Bit(Color.kBlack)
      )
    );

    // Add the intake mechanism
    intakeMech = intakePoint.append(
      new MechanismLigament2d(
        "intake",
        intake_LENGTH,
        0,
        intake_WIDTH,
        new Color8Bit(Color.kBlue)
      )
    );

    // Initialize visualization
    SmartDashboard.putData("intake Sim", mech);
  }

  @Override
  public void periodic() {
    // Update intake angle
    double currentAngleRad = intake.getSimulation().getAngleRads();
    intakeMech.setAngle(Units.radiansToDegrees(currentAngleRad));

    // Add telemetry data
    SmartDashboard.putNumber(
      "intake Angle (deg)",
      Units.radiansToDegrees(currentAngleRad)
    );
    SmartDashboard.putNumber(
      "intake Velocity (deg/s)",
      Units.radiansToDegrees(intake.getSimulation().getVelocityRadPerSec())
    );
    SmartDashboard.putNumber(
      "pivot Current (A)",
      intake.getSimulation().getCurrentDrawAmps()
    );
    SmartDashboard.putNumber(
      "spinner Velocity (rot/s)",
      intake.SpinnerGetVelocity()
    );

  }
}
