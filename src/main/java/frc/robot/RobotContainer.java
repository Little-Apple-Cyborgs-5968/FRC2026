// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.Console;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.Constants;
import frc.robot.commands.PathFindCommands;
import frc.robot.driverIO.ControllerRumble;
import frc.robot.driverIO.DashboardPublisher;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TurretShooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.simulation.IntakeSim;
import frc.robot.subsystems.simulation.TurretShooterSim;
import frc.robot.subsystems.RoboSingSubsystem;
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

    // for robot sing
    private final RoboSingSubsystem roboSing = new RoboSingSubsystem();

    // Turret subsystem
    private final TurretShooter turret = new TurretShooter();
    private final TurretShooterSim turretSim = new TurretShooterSim(turret);

    // Intake Subsystem

    private final Intake intake = new Intake();
    private final IntakeSim intakeSim = new IntakeSim(intake);

    // Auto chooser
    private final SendableChooser<Command> autoChooser;

    

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

        //creates and puts the auto chooser object from pathplanner autos
        autoChooser = AutoBuilder.buildAutoChooser();
        
        //puts auto chooser on smartdashboard for selection
        SmartDashboard.putData("Auto Chooser", autoChooser);

        dashboard = new DashboardPublisher(drivetrain, autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // robo sing controls
        joystick.start().and(joystick.a()).onTrue(roboSing.runOnce(roboSing::play));
        joystick.start().and(joystick.b()).onTrue(roboSing.runOnce(roboSing::stop));


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
        // Map bumpers/triggers to intake commands
        joystick.leftBumper().onTrue(
            intake.PivotSetAngleCommand(0)
        );

        joystick.rightBumper().onTrue(
            intake.PivotSetAngleCommand(45)
        );

        joystick.leftTrigger().onTrue(
            intake.SpinnerMoveAtVelocityCommand(0)
        );

        joystick.rightTrigger().onTrue(
            intake.SpinnerMoveAtVelocityCommand(getTunableValue())
        );

        // joystick.leftTrigger().onTrue(
        //     intake.DeployCommand()
        // );

        // joystick.leftBumper().onTrue(
        //     intake.StowCommand()
        // );

        joystick.x().onTrue(
            turret.setAngleCommand(90)
        );

        joystick.y().onTrue(
            turret.setAngleCommand(0)
        );

        // Auto-aim bindings
        // B button: Auto-aim at hub (stationary)
        joystick.b().whileTrue(
            turret.autoAimCommand(() -> drivetrain.getState().Pose, TurretUtil.TargetType.HUB)
        );

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
        joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    //** Called from Robot.java autonomousInit(), gets selected auto command */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }


    /** Called from Robot.java robotPeriodic(), updates dashboard */
    public void updateDashboard() {
        dashboard.update();
    }

    /** Gets the current tunable value from the dashboard - use this for testing/tuning! */
    public double getTunableValue() {
        return dashboard.getTunableValue();
    }
}
