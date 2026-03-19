package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeJiggleCommand extends Command {
  private final Intake intake;
  private final Timer timer;
  private boolean isAtFirstAngle;

  public IntakeJiggleCommand(Intake intake) {
    this.intake = intake;
    this.timer = new Timer();
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    isAtFirstAngle = true;
    intake.PivotSetAngle(Constants.Intake.kIntakeAngleJiggle1);
    intake.StopSpinner();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If the interval has passed, swap the target angle
    if (timer.hasElapsed(Constants.Intake.kJiggleInterval)) {
      isAtFirstAngle = !isAtFirstAngle;
      timer.restart();
      
      if (isAtFirstAngle) {
        intake.PivotSetAngle(Constants.Intake.kIntakeAngleJiggle1);
      } else {
        intake.PivotSetAngle(Constants.Intake.kIntakeAngleJiggle2);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Return to stowed position or leave it depending on desired behavior.
    // For safety, we can stop the pivot motor.
    intake.PivotStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Runs continuously until interrupted.
    return false;
  }
}
