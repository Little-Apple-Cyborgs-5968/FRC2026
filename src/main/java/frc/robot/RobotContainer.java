// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.generated.TunerConstants;
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
    
    // SYOMDrive (Synchronized Yaw-Optimized Motion Drive) - auto-rotates to face travel direction
    private boolean isSYOMDriveEnabled = false;
    
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    

    // ==================== DRIVER I/O ====================

    private final CommandXboxController joystick = new CommandXboxController(0);

    // Rumble command for driver feedback
    private final ControllerRumble rumble = new ControllerRumble(joystick);

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
    private final IntakeSim intakeSim = new IntakeSim(intake);

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

    

    public RobotContainer() {
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
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                // Check if SYOMDrive mode is enabled
                if (isSYOMDriveEnabled) {
                    // SYOMDrive: Calculate automatic rotation to face travel direction
                    double vx = -joystick.getLeftY() * MaxSpeed;
                    double vy = -joystick.getLeftX() * MaxSpeed;
                    
                    // Calculate velocity magnitude
                    double velocityMagnitude = Math.sqrt(vx * vx + vy * vy);
                    
                    // Calculate rotational rate
                    double rotationalRate = 0;
                    
                    if (velocityMagnitude > Constants.Swerve.kSYOMDriveMinVelocity) {
                        // Calculate the direction of travel (desired heading)
                        Rotation2d desiredHeading = new Rotation2d(vx, vy);
                        
                        // Get current heading
                        Rotation2d currentHeading = drivetrain.getState().Pose.getRotation();
                        
                        // Calculate heading error
                        double headingError = desiredHeading.minus(currentHeading).getRadians();
                        
                        // Normalize to [-π, π]
                        while (headingError > Math.PI) headingError -= 2 * Math.PI;
                        while (headingError < -Math.PI) headingError += 2 * Math.PI;
                        
                        // Apply proportional control for smooth rotation
                        rotationalRate = headingError * Constants.Swerve.kSYOMDriveRotationKp;
                    }
                    
                    return drive.withVelocityX(vx)
                                .withVelocityY(vy)
                                .withRotationalRate(rotationalRate);
                } else {
                    // Normal field-centric drive with manual rotation
                    return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
                }
            })
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

        // Example usage of PathFindCommands to go to preset locations
        // joystick.a().and(joystick.y()).onTrue(
        //     PathFindCommands.pathfindToPath("hub_front")
        // );

        // Example usage of rumble command (controller rumble for driver feedback)
        // Moved to start+back combo to avoid conflict with auto-aim
        joystick.start().and(joystick.back()).onTrue(
            rumble.doublePulse()
        );
        
        // Toggle SYOMDrive (Synchronized Yaw-Optimized Motion Drive) with A button
        // When enabled, robot automatically faces the direction it's traveling
        // joystick.a().onTrue(
        //     drivetrain.runOnce(() -> {
        //         isSYOMDriveEnabled = !isSYOMDriveEnabled;
        //         System.out.println("SYOMDrive " + (isSYOMDriveEnabled ? "ENABLED" : "DISABLED"));
        //     })
        // );
        
        // Map bumpers/triggers to intake commands

        joystick.leftTrigger().onTrue(
            intake.SpinnerStopCommand()
        );

        joystick.rightTrigger().onTrue(
            intake.SpinnerTunableCommand(dashboard)
        );

        joystick.leftBumper().onTrue(
            intake.PivotStopCommand()
        );
        joystick.rightBumper().onTrue(
            intake.PivotTunableCommand(dashboard)
        );

        joystick.leftBumper().onTrue(
            spindexer.stopCommand()
        );

        joystick.rightBumper().onTrue(
            spindexer.tunableCommand(dashboard)
        );

        joystick.leftBumper().whileTrue(
            feeder.stopCommand()
        );

        joystick.rightBumper().whileTrue(
            feeder.tunableCommand(dashboard)
        );

         joystick.leftBumper().whileTrue(
            shooter.stopCommand()
        );

        joystick.rightBumper().whileTrue(
             shooter.flywheelTunableCommand(dashboard)
        );
// // INTAKE PIVOT TEMP BINDINGS
//         joystick.leftBumper().onTrue(
//             intake.StowCommand()
//         );
//         joystick.rightBumper().onTrue(
//             intake.DeployCommand()
            
//         );
// // SPINDEXER TEMP BINDINGS
//         joystick.x().onTrue(
//             spindexer.stopCommand()
//         );

//         joystick.b().onTrue(
//             spindexer.runCommand()
//         );

// //SHOOTER TEMP BINDINGS
//         joystick.y().whileTrue(
//             shooter.runFlywheelsAtSpeedCommand(10)
//         );

//         joystick.a().onTrue(
//             shooter.setHoodAngleCommand(65)
//         );
// FEEDER TEMP BINDINGS
        // joystick.leftBumper().onTrue(
        //     feeder.stopCommand()
        // );

        // joystick.rightBumper().onTrue(
        //     feeder.tunableCommand(dashboard)
        // );

// SHOOTER TEMP BINDINGS
        // joystick.leftBumper().whileTrue(
        //     shooter.stopCommand()
        // );

        // joystick.rightBumper().onTrue(
        //      shooter.flywheelTunableCommand(dashboard)
        // );




        // Auto-aim bindings
        // B button: Auto-aim at hub (stationary)
        // joystick.b().whileTrue(
        //     turret.autoAimCommand(() -> drivetrain.getState().Pose, TurretUtil.TargetType.HUB)
        // );

        new Trigger(() -> Math.round(GameState.timeRemainingInCurrentState()) == 5).onTrue(rumble.lightPulse());
        new Trigger(() -> Math.round(GameState.timeRemainingInCurrentState()) == 0).onTrue(rumble.doublePulse());
        // joystick.b().onTrue(
        //     PathFindCommands.pathfindAndDo("score_barge", drivetrain.runOnce(() -> System.out.println("run reef motors")))
        // );
        // joystick.y().onTrue(
        //     PathFindCommands.pathfindToPath("still")
        // );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on back button press(back button)
        // Also rezeroes turret, intake pivot, and shooter hood encoders
        joystick.back().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
                .andThen(turret.RezeroCommand())
                .andThen(intake.RezeroCommand())
                .andThen(shooter.RezeroCommand())
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    //** Called from Robot.java autonomousInit(), gets selected auto command */
    public Command getAutonomousCommand() {
        return dashboard.getAuto();
    }


    /** Called from Robot.java robotPeriodic(), updates dashboard */
    public void updateDashboard() {
        dashboard.update();
        // Update SYOMDrive status on SmartDashboard
        SmartDashboard.putBoolean("DASHBOARD/SYOMDrive Enabled", isSYOMDriveEnabled);
    }

    /** Gets the current tunable value from the dashboard - use this for testing/tuning! */
    public double getTunableValue() {
        return dashboard.getTunableSetpoint1();
    }
}
