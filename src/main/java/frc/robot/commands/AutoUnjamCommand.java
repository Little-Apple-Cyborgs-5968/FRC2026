package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Spindexer;

/**
 * A command that automatically unjams the indexing mechanism.
 * Runs both the feeder and spindexer forward, but constantly monitors the spindexer's stator current.
 * If the current exceeds a configurable threshold (indicating a jam), this command briefly reverses
 * both mechanisms to clear the jam before resuming forward indexing.
 */
public class AutoUnjamCommand extends Command {
  private final Spindexer spindexer;
  private final Feeder feeder;
  private final Timer unjamTimer = new Timer();
  private final Timer stallTimer = new Timer();

  private boolean isUnjamming = false;

  public AutoUnjamCommand(Spindexer spindexer, Feeder feeder) {
    this.spindexer = spindexer;
    this.feeder = feeder;
    addRequirements(spindexer, feeder);
  }

  @Override
  public void initialize() {
    isUnjamming = false;
    unjamTimer.stop();
    unjamTimer.reset();
    stallTimer.start();
    stallTimer.reset();
  }

  @Override
  public void execute() {
    if (!isUnjamming) {
      // Normal operation (running forward)
      spindexer.setVelocity(Constants.Spindexer.kSpinnerSpeed);
      feeder.setVelocity(Constants.Feeder.kSpinnerSpeed);

      // Check for stall conditions: high current AND low velocity
      boolean isStalled = spindexer.getStatorCurrent() > Constants.Spindexer.kJamCurrentThresholdAmps
          && Math.abs(spindexer.getVelocity()) < Constants.Spindexer.kJamVelocityThresholdRPS;

      if (isStalled) {
        // If it has been stalled for the full debounce time, trigger unjam
        if (stallTimer.hasElapsed(Constants.Spindexer.kJamDebounceTimeSeconds)) {
          isUnjamming = true;
          unjamTimer.restart(); // start the unjam timer
        }
      } else {
        // Reset the stall timer if we recover before the debounce completes
        stallTimer.reset();
      }
    } else {
      // Unjam operation (running backward)
      spindexer.setVelocity(Constants.Spindexer.kUnjamSpindexerReverseSpeed);
      feeder.setVelocity(Constants.Feeder.kUnjamFeederReverseSpeed);

      // Check if unjam is done
      if (unjamTimer.hasElapsed(Constants.Spindexer.kUnjamDurationSeconds)) {
        isUnjamming = false;
        unjamTimer.stop();
        unjamTimer.reset();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    spindexer.setVelocity(0);
    feeder.setVelocity(0);
  }
}
