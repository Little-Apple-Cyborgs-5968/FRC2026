package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * DefaultShootCommand — A fixed-preset shot command that:
 * <ul>
 *   <li>Rotates the turret to face <b>directly forward</b>
 *       ({@link Constants.DefaultShot#kTurretAngleDegrees} = 0°)</li>
 *   <li>Spins the flywheels to
 *       {@link Constants.DefaultShot#kFlywheelSpeedRPS}</li>
 *   <li>Sets the hood to
 *       {@link Constants.DefaultShot#kHoodAngleDegrees}</li>
 * </ul>
 *
 * <p>All tunable values live in {@link Constants.DefaultShot} — change them
 * there without touching this class.
 *
 * <p>The command runs <em>continuously</em> (never finishes on its own), so it
 * is suitable as a {@code whileTrue()} binding or as a named PathPlanner command.
 *
 * <h3>Typical usage</h3>
 * <pre>{@code
 * // Hold a button to execute the default shot
 * operatorJoystick.a().whileTrue(new DefaultShootCommand(turret, shooter));
 *
 * // Register as a PathPlanner named command
 * NamedCommands.registerCommand("DEFAULT_SHOOT", new DefaultShootCommand(turret, shooter));
 * }</pre>
 */
public class DefaultShootCommand extends ParallelCommandGroup {

    /**
     * Creates a new DefaultShootCommand.
     *
     * <p>Composed of two parallel branches — one per subsystem:
     * <ol>
     *   <li>A single {@code Turret} {@code run()} that holds the turret at 0° every loop.</li>
     *   <li>A single {@code Shooter} {@code run()} that simultaneously sets both the
     *       flywheel speed and the hood angle every loop (same pattern as
     *       {@link Shooter#shotTunableCommand}).</li>
     * </ol>
     * Each branch requires exactly one subsystem, so there is no duplicate-requirement conflict.
     *
     * @param turret  The {@link Turret} subsystem
     * @param shooter The {@link Shooter} subsystem
     */
    public DefaultShootCommand(Turret turret, Shooter shooter) {
        addCommands(
            // Branch 1 — Turret: hold forward (0°) every loop.
            turret.run(() -> turret.setAngle(Constants.DefaultShot.kTurretAngleDegrees))
                  .withName("DefaultShot-Turret"),

            // Branch 2 — Shooter: set flywheel speed AND hood angle in a single run()
            // so only one command requires the Shooter subsystem (mirrors shotTunableCommand).
            shooter.shotTunableCommand(
                () -> Constants.DefaultShot.kFlywheelSpeedRPS,
                () -> Constants.DefaultShot.kHoodAngleDegrees
            ).withName("DefaultShot-Shooter")
        );

        setName("DefaultShootCommand");
    }
}
