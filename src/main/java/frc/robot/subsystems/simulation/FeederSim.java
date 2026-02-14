package frc.robot.subsystems.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Feeder;

/**
 * Visualization for the feeder subsystem in simulation.
 * Shows a pair of counter-rotating rollers feeding game pieces.
 */
public class FeederSim extends SubsystemBase {

  private final Feeder feeder;

  // Simulation display
  private final Mechanism2d mech;
  private final MechanismRoot2d leftRoot;
  private final MechanismRoot2d rightRoot;
  
  // Left roller (top)
  private final MechanismLigament2d leftRoller;
  private final MechanismLigament2d leftSpoke1;
  private final MechanismLigament2d leftSpoke2;
  
  // Right roller (bottom)
  private final MechanismLigament2d rightRoller;
  private final MechanismLigament2d rightSpoke1;
  private final MechanismLigament2d rightSpoke2;

  // Game piece visualization
  private final MechanismRoot2d gamePieceRoot;
  private final MechanismLigament2d gamePiece;

  // Visualization constants
  private final double ROLLER_RADIUS = 0.08;
  private final double ROLLER_WIDTH = 6;
  private final double SPOKE_WIDTH = 3;
  private final double ROLLER_SEPARATION = 0.25;
  private final double GAME_PIECE_SIZE = 0.12;

  // Animation tracking
  private double leftAngle = 0;
  private double rightAngle = 0;
  private double gamePieceX = 0.2;
  private boolean hasGamePiece = false;

  /**
   * Creates a new visualization for the feeder.
   *
   * @param feederSubsystem The feeder subsystem to visualize
   */
  public FeederSim(Feeder feederSubsystem) {
    this.feeder = feederSubsystem;

    // Create the simulation display
    mech = new Mechanism2d(1.0, 0.6);
    
    // Left roller (top) at 0.3, 0.4
    leftRoot = mech.getRoot("leftRollerRoot", 0.3, 0.4);
    
    leftRoller = leftRoot.append(
      new MechanismLigament2d(
        "LeftRoller",
        ROLLER_RADIUS,
        0,
        ROLLER_WIDTH,
        new Color8Bit(Color.kDarkBlue)
      )
    );
    
    leftSpoke1 = leftRoot.append(
      new MechanismLigament2d(
        "LeftSpoke1",
        ROLLER_RADIUS,
        0,
        SPOKE_WIDTH,
        new Color8Bit(Color.kLightBlue)
      )
    );
    
    leftSpoke2 = leftRoot.append(
      new MechanismLigament2d(
        "LeftSpoke2",
        ROLLER_RADIUS,
        180,
        SPOKE_WIDTH,
        new Color8Bit(Color.kLightBlue)
      )
    );

    // Right roller (bottom) at 0.3, 0.15
    rightRoot = mech.getRoot("rightRollerRoot", 0.3, 0.15);
    
    rightRoller = rightRoot.append(
      new MechanismLigament2d(
        "RightRoller",
        ROLLER_RADIUS,
        0,
        ROLLER_WIDTH,
        new Color8Bit(Color.kDarkBlue)
      )
    );
    
    rightSpoke1 = rightRoot.append(
      new MechanismLigament2d(
        "RightSpoke1",
        ROLLER_RADIUS,
        0,
        SPOKE_WIDTH,
        new Color8Bit(Color.kLightBlue)
      )
    );
    
    rightSpoke2 = rightRoot.append(
      new MechanismLigament2d(
        "RightSpoke2",
        ROLLER_RADIUS,
        180,
        SPOKE_WIDTH,
        new Color8Bit(Color.kLightBlue)
      )
    );

    // Game piece (ball)
    gamePieceRoot = mech.getRoot("gamePieceRoot", gamePieceX, 0.275);
    gamePiece = gamePieceRoot.append(
      new MechanismLigament2d(
        "GamePiece",
        GAME_PIECE_SIZE,
        0,
        10,
        new Color8Bit(Color.kOrange)
      )
    );

    // Initialize visualization
    SmartDashboard.putData("FEEDER/Feeder Sim", mech);
  }

  @Override
  public void periodic() {
    // Update roller angles based on velocity
    double velocity = feeder.getVelocity(); // rotations per second
    double deltaAngle = velocity * 360.0 * 0.02; // degrees per 20ms period
    
    // Left roller rotates forward, right roller rotates backward (counter-rotating)
    leftAngle += deltaAngle;
    rightAngle -= deltaAngle; // Opposite direction
    
    // Keep angles in [0, 360) range
    leftAngle = leftAngle % 360;
    rightAngle = rightAngle % 360;

    // Update left roller visualization
    leftRoller.setAngle(leftAngle);
    leftSpoke1.setAngle(leftAngle);
    leftSpoke2.setAngle(leftAngle + 180);

    // Update right roller visualization
    rightRoller.setAngle(rightAngle);
    rightSpoke1.setAngle(rightAngle);
    rightSpoke2.setAngle(rightAngle + 180);

    // Animate game piece movement when feeder is running
    if (Math.abs(velocity) > 1.0) {
      hasGamePiece = true;
      // Move game piece through feeder (left to right when positive velocity)
      gamePieceX += (velocity > 0 ? 0.01 : -0.01);
      
      // Reset position when it exits
      if (gamePieceX > 0.8) {
        gamePieceX = 0.2;
      } else if (gamePieceX < 0.2) {
        gamePieceX = 0.8;
      }
      
      gamePieceRoot.setPosition(gamePieceX, 0.275);
    } else {
      // Hide game piece when not running
      hasGamePiece = false;
    }

    // Update game piece visibility and color
    if (hasGamePiece) {
      gamePiece.setColor(new Color8Bit(Color.kOrange));
    } else {
      gamePiece.setColor(new Color8Bit(Color.kBlack)); // Hidden when not running
    }

    // Color coding for rollers based on velocity
    Color8Bit spokeColor;
    if (Math.abs(velocity) < 0.5) {
      spokeColor = new Color8Bit(Color.kGray); // Stopped
    } else if (velocity > 0) {
      spokeColor = new Color8Bit(Color.kGreen); // Forward (feeding)
    } else {
      spokeColor = new Color8Bit(Color.kRed); // Reverse (ejecting)
    }

    leftSpoke1.setColor(spokeColor);
    leftSpoke2.setColor(spokeColor);
    rightSpoke1.setColor(spokeColor);
    rightSpoke2.setColor(spokeColor);

    // Add telemetry data
    SmartDashboard.putNumber("FEEDER/Velocity (RPS)", velocity);
    SmartDashboard.putNumber("FEEDER/Target Velocity (RPS)", feeder.getTargetVelocity());
    SmartDashboard.putNumber("FEEDER/Voltage (V)", feeder.getVoltage());
    SmartDashboard.putNumber("FEEDER/Current (A)", feeder.getCurrent());
    SmartDashboard.putNumber("FEEDER/Temperature (C)", feeder.getTemperature());
    SmartDashboard.putBoolean("FEEDER/Is Running", feeder.isRunning());
    SmartDashboard.putBoolean("FEEDER/At Target", feeder.atTargetVelocity());
    SmartDashboard.putNumber(
      "FEEDER/Sim Current Draw (A)",
      feeder.getSimulation().getCurrentDrawAmps()
    );
  }
}
