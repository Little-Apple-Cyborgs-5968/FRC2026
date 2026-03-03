// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.generated.TunerConstants;
import frc.robot.commands.PathFindCommands;
import frc.robot.commands.SYOMDriveCommand;
// import frc.robot.commands.WhiskerPickupCommand;
import frc.robot.driverIO.ControllerRumble;
import frc.robot.driverIO.DashboardPublisher;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.simulation.FeederSim;
import frc.robot.subsystems.simulation.IntakeSim;
import frc.robot.subsystems.simulation.ShooterSim;
import frc.robot.subsystems.simulation.SpindexerSim;
import frc.robot.subsystems.simulation.TurretSim;
import frc.robot.subsystems.simulation.Visualizer;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.GameState;
import frc.robot.utils.TurretUtil;


public class RobotContainer {
    private double robotCentricDriveSpeed = Constants.Swerve.kRobotCentricSpeed;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    //field centric drive command object
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // robot centric drive command object
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    // Shoot-on-the-move toggle — right trigger turns on lead-compensated warm-up; releasing fires; second press cancels
    private boolean isShootOnMoveActive = false;

    // Whisker pickup mode toggle — B button cycles on/off
    private boolean isPickupModeActive = false;
    
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    

    // ==================== DRIVER I/O ====================

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    // Rumble command for driver feedback
    private final ControllerRumble rumble = new ControllerRumble(joystick);
    private final ControllerRumble operatorRumble = new ControllerRumble(operatorJoystick);

    // Dashboard publisher
    private final DashboardPublisher dashboard;

    // ==================== SUBSYSTEMS ====================
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    // Vision subsystem for AprilTag localization
    private final Vision vision;

    // Turret subsystem
    private final Turret turret = new Turret();
    private final TurretSim turretSim = new TurretSim(turret);

    // Intake Subsystem
    private final Intake intake = new Intake();
    //private final IntakeSim intakeSim = new IntakeSim(intake);

    // Spindexer Subsystem
    private final Spindexer spindexer = new Spindexer();
    private final SpindexerSim spindexerSim = new SpindexerSim(spindexer);

    // Feeder Subsystem
    private final Feeder feeder = new Feeder();
    private final FeederSim feederSim = new FeederSim(feeder);

    // Shooter Subsystem
    private final Shooter shooter = new Shooter();
    private final ShooterSim shooterSim = new ShooterSim(shooter);

    
    private final Visualizer visualizer = new Visualizer(turret,shooter);

    // SYOMDrive command instance — toggled on/off with the X button
    private final SYOMDriveCommand syomDriveCommand = new SYOMDriveCommand(
            drivetrain,
            () -> -joystick.getLeftY() * MaxSpeed,
            () -> -joystick.getLeftX() * MaxSpeed,
            MaxSpeed,
            MaxAngularRate);

    

    public RobotContainer() {
        // Register named commands for PathPlanner autos

        NamedCommands.registerCommand("INTAKE_DEPLOY", intake.DeployCommand());
        NamedCommands.registerCommand("INTAKE_STOW", intake.StowCommand());
        NamedCommands.registerCommand("INTAKE_STOP_SPINNER", intake.SpinnerStopCommand());

        NamedCommands.registerCommand("SPINDEXER_RUN", spindexer.runCommand());
        NamedCommands.registerCommand("SPINDEXER_STOP", spindexer.stopCommand());


        NamedCommands.registerCommand("FEEDER_RUN", feeder.runCommand());
        NamedCommands.registerCommand("FEEDER_STOP", feeder.stopCommand());
        
        NamedCommands.registerCommand("HOOD_TRENCH", shooter.setHoodToTrenchCommand());
        NamedCommands.registerCommand("SHOOTER_RUN", shooter.runFlywheelsCommand());

        // Shoot-on-the-move warm-up: turret + flywheel only (lead-compensated, no feeder/spindexer)
        NamedCommands.registerCommand("SHOOTER_WARMUP",
            turret.shootOnMoveCommandTurret(
                () -> drivetrain.getState().Pose,
                () -> ChassisSpeeds.fromRobotRelativeSpeeds(
                        drivetrain.getState().Speeds,
                        drivetrain.getState().Pose.getRotation()),
                TurretUtil.TargetType.HUB
            ).alongWith(
                shooter.shootOnMoveFlywheelOnlyCommand(
                    () -> drivetrain.getState().Pose,
                    () -> ChassisSpeeds.fromRobotRelativeSpeeds(
                            drivetrain.getState().Speeds,
                            drivetrain.getState().Pose.getRotation()),
                    TurretUtil.TargetType.HUB
                )
            ).withName("ShootOnMove-WarmUp")
        );

        // Shoot-on-the-move full shot: turret + hood + flywheel + spindexer + feeder (lead-compensated)
        NamedCommands.registerCommand("SHOOT_ON_MOVE",
            turret.shootOnMoveCommandTurret(
                () -> drivetrain.getState().Pose,
                () -> ChassisSpeeds.fromRobotRelativeSpeeds(
                        drivetrain.getState().Speeds,
                        drivetrain.getState().Pose.getRotation()),
                TurretUtil.TargetType.HUB
            ).alongWith(
                shooter.shootOnMoveCommandShooter(
                    () -> drivetrain.getState().Pose,
                    () -> ChassisSpeeds.fromRobotRelativeSpeeds(
                            drivetrain.getState().Speeds,
                            drivetrain.getState().Pose.getRotation()),
                    TurretUtil.TargetType.HUB
                ),
                spindexer.runCommand(),
                feeder.runCommand()
            ).withName("ShootOnMove-Firing")
        );
        // Initialize Vision subsystem with drivetrain integration
        vision = new Vision(
            // Pose supplier - gets current robot pose from drivetrain
            () -> drivetrain.getState().Pose,
            // Heading supplier - gets current robot heading for MegaTag2
            () -> drivetrain.getState().Pose.getRotation(),
            // Vision consumer - feeds vision measurements to drivetrain pose estimator
            (pose, timestamp, stdDevs) -> drivetrain.addVisionMeasurement(pose, timestamp, stdDevs)
        );

        // Dashboard now handles auto chooser creation and publishing
        dashboard = new DashboardPublisher(drivetrain);

        configureBindings();

        // Register drivetrain pose supplier with turret for distance-to-target telemetry
        turret.setPoseSupplier(() -> drivetrain.getState().Pose);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                     .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                     .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        // robot oriented drive forwad and backward, also left right
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            driveRobotCentric.withVelocityX(robotCentricDriveSpeed).withVelocityY(0))
        );
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() -> 
            driveRobotCentric.withVelocityX(0).withVelocityY(-robotCentricDriveSpeed))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> 
            driveRobotCentric.withVelocityX(-robotCentricDriveSpeed).withVelocityY(0))
        );
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() -> 
            driveRobotCentric.withVelocityX(0).withVelocityY(robotCentricDriveSpeed))
        );  

        // Both bumpers together: stow intake
        joystick.rightBumper().and(joystick.leftBumper()).onTrue(
            intake.StowCommand()
        );

        // Left bumper only (not right): stop spinner
        joystick.leftBumper().and(joystick.rightBumper().negate()).onTrue(
            intake.SpinnerStopCommand()
        );

        // Right bumper only (not left): deploy intake
        joystick.rightBumper().and(joystick.leftBumper().negate()).onTrue(
            intake.DeployCommand()
        );

        shooter.setDefaultCommand(shooter.stopCommand());
        spindexer.setDefaultCommand(spindexer.stopCommand());
        feeder.setDefaultCommand(feeder.stopCommand());

        // ── RIGHT TRIGGER: Toggle shoot mode ──────────────────────────────────────
        //
        // PRESS 1 (toggle ON, trigger held):
        //   • Turret   → auto-aim continuously
        //   • Flywheel → spun to correct speed (flywheel-only, no hood movement)
        //   • Hood     → stays put
        //   • Feeder / Spindexer → NOT running yet
        //
        // PRESS 1 RELEASE (shoot mode still ON):
        //   • Turret + Flywheel + Hood → all auto-aim continuously
        //   • Feeder + Spindexer → start running (balls feed into shooter)
        //   Continues until toggled off.
        //
        // PRESS 2 (toggle OFF):
        //   • Everything stops (default commands take over)
        //   • Hood moves to trench position

        // Toggle the flag on every right-trigger press (shoot-on-the-move)
        joystick.rightTrigger().onTrue(
            Commands.runOnce(() -> {
                isShootOnMoveActive = !isShootOnMoveActive;
                SmartDashboard.putBoolean("DASHBOARD/Shoot On Move Active", isShootOnMoveActive);
            })
        );

        // Derive Trigger objects from the flag — the scheduler reacts to these every loop
        Trigger shootOnMoveOn   = new Trigger(() -> isShootOnMoveActive);
        Trigger triggerHeld     = joystick.rightTrigger();
        Trigger warmUpActive    = shootOnMoveOn.and(triggerHeld);         // ON + held  → warm up
        Trigger firingActive    = shootOnMoveOn.and(triggerHeld.negate()); // ON + released → fire



        // Warm-up phase: turret + flywheel only, both lead-compensated
        warmUpActive.whileTrue(
            turret.shootOnMoveCommandTurret(
                () -> drivetrain.getState().Pose,
                () -> ChassisSpeeds.fromRobotRelativeSpeeds(
                        drivetrain.getState().Speeds,
                        drivetrain.getState().Pose.getRotation()),
                TurretUtil.TargetType.HUB
            ).alongWith(
                shooter.shootOnMoveFlywheelOnlyCommand(
                    () -> drivetrain.getState().Pose,
                    () -> ChassisSpeeds.fromRobotRelativeSpeeds(
                            drivetrain.getState().Speeds,
                            drivetrain.getState().Pose.getRotation()),
                    TurretUtil.TargetType.HUB
                )
            ).withName("ShootOnMove-WarmUp")
        );

        // Firing phase: turret + full shooter (hood+flywheel) + spindexer + feeder, all lead-compensated
        firingActive.whileTrue(
            turret.shootOnMoveCommandTurret(
                () -> drivetrain.getState().Pose,
                () -> ChassisSpeeds.fromRobotRelativeSpeeds(
                        drivetrain.getState().Speeds,
                        drivetrain.getState().Pose.getRotation()),
                TurretUtil.TargetType.HUB
            ).alongWith(
                shooter.shootOnMoveCommandShooter(
                    () -> drivetrain.getState().Pose,
                    () -> ChassisSpeeds.fromRobotRelativeSpeeds(
                            drivetrain.getState().Speeds,
                            drivetrain.getState().Pose.getRotation()),
                    TurretUtil.TargetType.HUB
                ),
                spindexer.runCommand(),
                feeder.runCommand()
            ).withName("ShootOnMove-Firing")
        );

        // When shoot-on-the-move mode is toggled OFF, move hood to trench position
        shootOnMoveOn.onFalse(
            shooter.setHoodToTrenchCommand()
        );

        joystick.start().whileTrue(
            spindexer.reverseCommand().alongWith(feeder.reverseCommand())
        );



        // ── X BUTTON: Toggle SYOMDrive (Synchronized Yaw-Optimized Motion Drive) ─
        // Press once → robot auto-rotates to face travel direction.
        // Press again → returns to normal field-centric drive with manual rotation.
        joystick.b().toggleOnTrue(syomDriveCommand);

        
        // Pathfind to nearest trench shoot pose; cancelled by pressing either joystick stick

        joystick.a().and(joystick.y()).onTrue(
            Commands.defer(() -> {
                String zone = dashboard.getPathfindZone();
                switch (zone) {
                    case "Mid":
                        return PathFindCommands.pathfindToNearestPose(
                            () -> drivetrain.getState().Pose,
                            List.of(FieldConstants.getLeftNeutral(), FieldConstants.getRightNeutral())
                        ).until(joystick.leftStick().or(joystick.rightStick()));
                    case "Opp":
                        return PathFindCommands.pathfindToNearestPose(
                            () -> drivetrain.getState().Pose,
                            List.of(FieldConstants.getLeftOpp(), FieldConstants.getRightOpp())
                        ).until(joystick.leftStick().or(joystick.rightStick()));
                    case "Home":
                    default:
                        return PathFindCommands.pathfindToNearestPose(
                            () -> drivetrain.getState().Pose,
                            List.of(FieldConstants.getLeftTrenchShoot(), FieldConstants.getRightTrenchShoot())
                        ).until(joystick.leftStick().or(joystick.rightStick()));
                }
            }, java.util.Set.of(drivetrain))
        );

        joystick.a().onTrue(
            shooter.setHoodToTrenchCommand()

        );



        // reset the field-centric heading on back button press(back button)
        joystick.back().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        );


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );


        new Trigger(() -> Math.round(GameState.timeRemainingInCurrentState()) == 5).onTrue(rumble.lightPulse());
        new Trigger(() -> Math.round(GameState.timeRemainingInCurrentState()) == 0).onTrue(rumble.doublePulse());
        




        drivetrain.registerTelemetry(logger::telemeterize);

//=============================================================================================================
        //COMMENTED OUT 
//=============================================================================================================

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        // // Rumble when flywheel reaches target speed (rising edge only — one buzz per spin-up)
        // new Trigger(shooter::isFlywheelAtSpeed).onTrue(
        //     rumble.lightPulse()
        // );


        //SHOT TUNING ON Y BUTTON 
                // // ── B BUTTON: Run spindexer + feeder while held ───────────────────────────
        // joystick.b().whileTrue(
        //     spindexer.runCommand().alongWith(feeder.runCommand())
        // );

        // Y button: shot tunable — setpoint1 = flywheel speed (RPS), setpoint2 = hood angle (deg)
        // Suppliers are evaluated every loop so dashboard changes take effect immediately.
        // joystick.y().whileTrue(
        //     shooter.shotTunableCommand(
        //         dashboard::getTunableSetpoint1,
        //         dashboard::getTunableSetpoint2
        //     )
        // );


        //DRIVE STUFF

        // B button: snap to nearest 45° and hold while held.
        // // Uses the same deadband (10 % of MaxSpeed) as the default drive command.
        // joystick.b().whileTrue(
        //     SetYawCommand.snapToNearest45(
        //         drivetrain,
        //         () -> -MathUtil.applyDeadband(joystick.getLeftY(), 0.1) * MaxSpeed,
        //         () -> -MathUtil.applyDeadband(joystick.getLeftX(), 0.1) * MaxSpeed,
        //         MaxSpeed
        // );

        // ── B BUTTON: Toggle whisker ball-pickup mode ─────────────────────────────
        // Press once → robot drives forward using the whisker algorithm to collect Fuel balls.
        // Press again → stops and returns drivetrain to default teleop command.
        // joystick.b().onTrue(
        //     Commands.runOnce(() -> {
        //         isPickupModeActive = !isPickupModeActive;
        //         SmartDashboard.putBoolean("DASHBOARD/Pickup Mode Active", isPickupModeActive);
        //     })
        // );

        // Trigger pickupModeOn = new Trigger(() -> isPickupModeActive);
        // pickupModeOn.whileTrue(new WhiskerPickupCommand(drivetrain, vision));
    }

    //** Called from Robot.java autonomousInit(), gets selected auto command */
    public Command getAutonomousCommand() {
        return dashboard.getAuto();
    }


    /** Called from Robot.java robotPeriodic(), updates dashboard */
    public void updateDashboard() {
        dashboard.update();
        // Update SYOMDrive status on SmartDashboard
        SmartDashboard.putBoolean("DASHBOARD/SYOMDrive Enabled", syomDriveCommand.isScheduled());
        // Update shoot-on-the-move mode toggle status on SmartDashboard
        SmartDashboard.putBoolean("DASHBOARD/Shoot On Move Active", isShootOnMoveActive);
        // Update whisker pickup mode toggle status on SmartDashboard
        SmartDashboard.putBoolean("DASHBOARD/Pickup Mode Active", isPickupModeActive);
    }

    /** Gets the current tunable value from the dashboard - use this for testing/tuning! */
    public double getTunableValue() {
        return dashboard.getTunableSetpoint1();
    }
}
