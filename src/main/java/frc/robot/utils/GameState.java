package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;

public class GameState {
  public enum States {
    AUTO,
    TRANSITION,
    SHIFTONE,
    SHIFTTWO,
    SHIFTTHREE,
    SHIFTFOUR,
    ENDGAME,
    BLUE,
    RED,
    STANDING
  }

  // ...existing code...

  /**
   * Returns the time remaining in the current game state (auto, transition, shift, endgame).
   * Returns 0 if not in a timed state.
   */
  public static double timeRemainingInCurrentState() {
    States currentState = determineGameState();
    double timeRemaining = DriverStation.getMatchTime();
    switch (currentState) {
      case AUTO:
        // AUTO: 2:20 to 2:00 (150 to 120)
        return timeRemaining;
      case TRANSITION:
        // TRANSITION: 2:00 to 2:10 (120 to 130)
        return Math.max(0, timeRemaining - 120);
      case SHIFTONE:
        // SHIFTONE: 2:10 to 1:45 (130 to 105)
        return Math.max(0, timeRemaining - 105);
      case SHIFTTWO:
        // SHIFTTWO: 1:45 to 1:20 (105 to 80)
        return Math.max(0, timeRemaining - 80);
      case SHIFTTHREE:
        // SHIFTTHREE: 1:20 to 0:55 (80 to 55)
        return Math.max(0, timeRemaining - 55);
      case SHIFTFOUR:
        // SHIFTFOUR: 0:55 to 0:30 (55 to 30)
        return Math.max(0, timeRemaining - 30);
      case ENDGAME:
        // ENDGAME: 0:30 to 0:00 (30 to 0)
        return Math.max(0, timeRemaining);
      default:
        return 0;
    }
  }

  String gameData = DriverStation.getGameSpecificMessage();

  public States getActiveHub() {
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          return States.BLUE;
        case 'R':
          return States.RED;
        default:
          break;
      }
    } else {
      System.out.println("Code for no data received yet");
    }

    return null;
  }

  public static States determineGameState() {
    // Check autonomous first (2:20 to 2:00)
    if (DriverStation.isAutonomous()) {
      return States.AUTO;
    }

    // Check if we're not enabled
    if (!DriverStation.isTeleop()) {
      return States.STANDING;
    }

    // Teleop time checks (match time counts DOWN from 2:30)
    double timeRemaining = DriverStation.getMatchTime();

    // FRC 2026 timings:
    // TRANSITION: 2:00-2:10 (120 < t <= 130)
    // SHIFTONE: 2:10-1:45 (105 < t <= 130)
    // SHIFTTWO: 1:45-1:20 (80 < t <= 105)
    // SHIFTTHREE: 1:20-0:55 (55 < t <= 80)
    // SHIFTFOUR: 0:55-0:30 (30 < t <= 55)
    // ENDGAME: 0:30-0:00 (0 < t <= 30)

    if (timeRemaining > 120 && timeRemaining <= 130) {
      return States.TRANSITION;
    } else if (timeRemaining > 105 && timeRemaining <= 130) {
      return States.SHIFTONE;
    } else if (timeRemaining > 80 && timeRemaining <= 105) {
      return States.SHIFTTWO;
    } else if (timeRemaining > 55 && timeRemaining <= 80) {
      return States.SHIFTTHREE;
    } else if (timeRemaining > 30 && timeRemaining <= 55) {
      return States.SHIFTFOUR;
    } else if (timeRemaining > 0 && timeRemaining <= 30) {
      return States.ENDGAME;
    }

    return States.STANDING;
  }
}