package frc.robot.driverIO;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Utility class for controller rumble/haptic feedback.
 * Provides various rumble patterns for driver feedback.
 */
public class ControllerRumble {

    // ==================== RUMBLE INTENSITIES ====================
    public static final double INTENSITY_LIGHT = 0.3;
    public static final double INTENSITY_MEDIUM = 0.6;
    public static final double INTENSITY_STRONG = 1.0;

    // ==================== RUMBLE DURATIONS (seconds) ====================
    public static final double DURATION_SHORT = 0.15;
    public static final double DURATION_MEDIUM = 0.3;
    public static final double DURATION_LONG = 0.5;

    private final CommandXboxController controller;

    /**
     * Creates a new ControllerRumble instance.
     * 
     * @param controller The Xbox controller to rumble
     */
    public ControllerRumble(CommandXboxController controller) {
        this.controller = controller;
    }

    // ==================== BASIC RUMBLE METHODS ====================

    /**
     * Sets rumble on both sides of the controller.
     * 
     * @param intensity Rumble intensity (0.0 to 1.0)
     */
    public void setRumble(double intensity) {
        controller.getHID().setRumble(RumbleType.kBothRumble, intensity);
    }

    /**
     * Sets rumble on the left side of the controller.
     * 
     * @param intensity Rumble intensity (0.0 to 1.0)
     */
    public void setLeftRumble(double intensity) {
        controller.getHID().setRumble(RumbleType.kLeftRumble, intensity);
    }

    /**
     * Sets rumble on the right side of the controller.
     * 
     * @param intensity Rumble intensity (0.0 to 1.0)
     */
    public void setRightRumble(double intensity) {
        controller.getHID().setRumble(RumbleType.kRightRumble, intensity);
    }

    /**
     * Stops all rumble on the controller.
     */
    public void stopRumble() {
        controller.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    // ==================== RUMBLE COMMANDS ====================

    /**
     * Creates a command that rumbles the controller for a specified duration.
     * 
     * @param intensity Rumble intensity (0.0 to 1.0)
     * @param duration  Duration in seconds
     * @return Command that performs the rumble
     */
    public Command rumbleCommand(double intensity, double duration) {
        return Commands.runOnce(() -> setRumble(intensity))
            .andThen(Commands.waitSeconds(duration))
            .finallyDo(() -> stopRumble());
    }

    /**
     * Creates a command that rumbles only the left side for a specified duration.
     * 
     * @param intensity Rumble intensity (0.0 to 1.0)
     * @param duration  Duration in seconds
     * @return Command that performs the left rumble
     */
    public Command leftRumbleCommand(double intensity, double duration) {
        return Commands.runOnce(() -> setLeftRumble(intensity))
            .andThen(Commands.waitSeconds(duration))
            .finallyDo(() -> stopRumble());
    }

    /**
     * Creates a command that rumbles only the right side for a specified duration.
     * 
     * @param intensity Rumble intensity (0.0 to 1.0)
     * @param duration  Duration in seconds
     * @return Command that performs the right rumble
     */
    public Command rightRumbleCommand(double intensity, double duration) {
        return Commands.runOnce(() -> setRightRumble(intensity))
            .andThen(Commands.waitSeconds(duration))
            .finallyDo(() -> stopRumble());
    }

    // ==================== PRESET RUMBLE PATTERNS ====================

    /**
     * Quick light pulse - good for minor notifications.
     * 
     * @return Command for light pulse rumble
     */
    public Command lightPulse() {
        return rumbleCommand(INTENSITY_LIGHT, DURATION_SHORT);
    }

    /**
     * Medium pulse - good for action confirmations.
     * 
     * @return Command for medium pulse rumble
     */
    public Command mediumPulse() {
        return rumbleCommand(INTENSITY_MEDIUM, DURATION_MEDIUM);
    }

    /**
     * Strong pulse - good for important alerts.
     * 
     * @return Command for strong pulse rumble
     */
    public Command strongPulse() {
        return rumbleCommand(INTENSITY_STRONG, DURATION_MEDIUM);
    }

    /**
     * Double pulse pattern - good for success/completion feedback.
     * 
     * @return Command for double pulse rumble
     */
    public Command doublePulse() {
        return rumbleCommand(INTENSITY_MEDIUM, DURATION_SHORT)
            .andThen(Commands.waitSeconds(0.1))
            .andThen(rumbleCommand(INTENSITY_MEDIUM, DURATION_SHORT));
    }

    /**
     * Triple pulse pattern - good for critical alerts.
     * 
     * @return Command for triple pulse rumble
     */
    public Command triplePulse() {
        return rumbleCommand(INTENSITY_STRONG, DURATION_SHORT)
            .andThen(Commands.waitSeconds(0.1))
            .andThen(rumbleCommand(INTENSITY_STRONG, DURATION_SHORT))
            .andThen(Commands.waitSeconds(0.1))
            .andThen(rumbleCommand(INTENSITY_STRONG, DURATION_SHORT));
    }

    /**
     * Alternating left-right rumble pattern.
     * 
     * @param cycles Number of left-right cycles
     * @return Command for alternating rumble
     */
    public Command alternatingRumble(int cycles) {
        Command pattern = Commands.none();
        for (int i = 0; i < cycles; i++) {
            pattern = pattern
                .andThen(leftRumbleCommand(INTENSITY_MEDIUM, DURATION_SHORT))
                .andThen(rightRumbleCommand(INTENSITY_MEDIUM, DURATION_SHORT));
        }
        return pattern;
    }

    /**
     * Escalating rumble - increases intensity over time.
     * Good for countdown or warning feedback.
     * 
     * @return Command for escalating rumble
     */
    public Command escalatingRumble() {
        return rumbleCommand(INTENSITY_LIGHT, DURATION_SHORT)
            .andThen(Commands.waitSeconds(0.1))
            .andThen(rumbleCommand(INTENSITY_MEDIUM, DURATION_SHORT))
            .andThen(Commands.waitSeconds(0.1))
            .andThen(rumbleCommand(INTENSITY_STRONG, DURATION_MEDIUM));
    }

    /**
     * Continuous rumble that must be manually stopped.
     * 
     * @param intensity Rumble intensity (0.0 to 1.0)
     * @return Command that starts continuous rumble
     */
    public Command startContinuousRumble(double intensity) {
        return Commands.runOnce(() -> setRumble(intensity));
    }

    /**
     * Stops any ongoing rumble.
     * 
     * @return Command that stops rumble
     */
    public Command stopRumbleCommand() {
        return Commands.runOnce(this::stopRumble);
    }

    // ==================== GAME-SPECIFIC PATTERNS ====================

    /**
     * Rumble pattern for when a game piece is acquired.
     * 
     * @return Command for intake confirmation
     */
    public Command intakeConfirmation() {
        return doublePulse();
    }

    /**
     * Rumble pattern for when a shot is fired.
     * 
     * @return Command for shot confirmation
     */
    public Command shotConfirmation() {
        return strongPulse();
    }

    /**
     * Rumble pattern for alignment/ready state.
     * 
     * @return Command for alignment confirmation
     */
    public Command alignmentReady() {
        return lightPulse();
    }

    /**
     * Warning rumble for when time is running low.
     * 
     * @return Command for time warning
     */
    public Command timeWarning() {
        return triplePulse();
    }

    /**
     * Rumble pattern for autonomous mode start.
     * 
     * @return Command for auto start notification
     */
    public Command autoStartNotification() {
        return escalatingRumble();
    }
}
