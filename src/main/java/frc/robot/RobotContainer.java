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
import frc.robot.commands.AutoUnjamCommand;
import frc.robot.commands.DefaultShootCommand;
import frc.robot.commands.DrivetrainShootOnMoveCommand;
import frc.robot.commands.IntakeJiggleCommand;
import frc.robot.commands.PathFindCommands;
import frc.robot.commands.SYOMDriveCommand;
import frc.robot.commands.DefenseDriveCommand;
import frc.robot.utils.FieldZoneUtil.FieldZones;

import frc.robot.driverIO.ControllerRumble;
import frc.robot.driverIO.DashboardPublisher;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.BallCounter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.simulation.ClimberSim;
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

  // Swerve shoot-on-the-move toggle — left trigger rotates drivetrain to aim instead of turret
  private boolean isSwerveSOMActive = false;

  // Whisker pickup mode toggle — B button cycles on/off
  private boolean isPickupModeActive = false;

  // Pathfind zone cycle — operator left/right bumper
  private static final String[] PATHFIND_ZONES = {
    "Home",
    "Mid",
    "Opp",
    "Sweep"
  };
  private int pathfindZoneIndex = 0; // starts at "Home"

  // Shoot mode cycle — operator left/right trigger
  private static final String[] SHOOT_MODES = {
    "Hub",
    "Pass"
  };
  private int shootModeIndex = 0; // starts at "Hub"

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // ==================== DRIVER I/O ====================

  private final CommandXboxController joystick = new CommandXboxController(0);

  private final CommandXboxController operatorJoystick = new CommandXboxController(1);

  private final CommandXboxController testJoystick = new CommandXboxController(2);

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
  private final IntakeSim intakeSim = new IntakeSim(intake);

  // Spindexer Subsystem
  private final Spindexer spindexer = new Spindexer();
  private final SpindexerSim spindexerSim = new SpindexerSim(spindexer);

  // Feeder Subsystem 
  private final Feeder feeder = new Feeder();
  //private final FeederSim feederSim = new FeederSim(feeder);

  // Ball Counter Subsystem
  private final BallCounter ballCounter = new BallCounter();

  // Shooter Subsystem
  private final Shooter shooter = new Shooter();
  private final ShooterSim shooterSim = new ShooterSim(shooter);

  //Climber subsystem 
  private final Climber climberBS = new Climber();
  //private final ClimberSim climberSim = new ClimberSim(climber);

  private final Visualizer visualizer = new Visualizer(turret, shooter, climberBS, spindexer, feeder, drivetrain, intake);

  // Defense Drive command instance — toggled on/off with the right stick button
  private final DefenseDriveCommand defenseDriveCommand = new DefenseDriveCommand(
    drivetrain,
    intake,
    turret,
    () -> -joystick.getLeftY() * MaxSpeed,
    () -> -joystick.getLeftX() * MaxSpeed,
    () -> -joystick.getRightX() * MaxAngularRate,
    MaxSpeed,
    MaxAngularRate);

  public RobotContainer() {
    // Register named commands for PathPlanner autos

    NamedCommands.registerCommand("INTAKE_DEPLOY", intake.DeployCommand().withTimeout(0.01)); // Add a short timeout to ensure the deploy command finishes before any subsequent commands that might rely on the intake being deployed  
    NamedCommands.registerCommand("INTAKE_STOW", intake.StowCommand().withTimeout(0.01)); // Add a short timeout to ensure the stow command finishes before any subsequent commands that might rely on the intake being stowed
    NamedCommands.registerCommand("INTAKE_STOP_SPINNER", intake.SpinnerStopCommand());
    NamedCommands.registerCommand("INTAKE_JIGGLE", new IntakeJiggleCommand(intake));
    NamedCommands.registerCommand("SPINDEXER_CHILL_JIGGLE", spindexer.chillJiggleCommand());
    NamedCommands.registerCommand("SPIN_INTAKE", intake.SpinnerMoveAtVelocityCommand(Constants.Intake.kSpinnerSpeed));

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
        this::getShootTargetType
      ).alongWith(
        shooter.shootOnMoveFlywheelOnlyCommand(
          () -> drivetrain.getState().Pose,
          () -> ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds,
            drivetrain.getState().Pose.getRotation()),
          this::getShootTargetType
        )
      ).withName("ShootOnMove-WarmUp")
    );

    // Shoot-on-the-move warm-up with hood: turret + flywheel + hood (lead-compensated, no feeder/spindexer)
    // Use this variant when you want the hood pre-positioned at the correct firing angle during warm-up.
    NamedCommands.registerCommand("SHOOTER_FULL_WARMUP",
      turret.shootOnMoveCommandTurret(
        () -> drivetrain.getState().Pose,
        () -> ChassisSpeeds.fromRobotRelativeSpeeds(
          drivetrain.getState().Speeds,
          drivetrain.getState().Pose.getRotation()),
        this::getShootTargetType
      ).alongWith(
        shooter.shootOnMoveCommandShooter(
          () -> drivetrain.getState().Pose,
          () -> ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds,
            drivetrain.getState().Pose.getRotation()),
          this::getShootTargetType
        )
      ).withName("ShootOnMove-WarmUp-WithHood")
    );

    // Shoot-on-the-move full shot: turret + hood + flywheel + spindexer + feeder (lead-compensated)
    NamedCommands.registerCommand("SHOOT_ON_MOVE",
      turret.shootOnMoveCommandTurret(
        () -> drivetrain.getState().Pose,
        () -> ChassisSpeeds.fromRobotRelativeSpeeds(
          drivetrain.getState().Speeds,
          drivetrain.getState().Pose.getRotation()),
        this::getShootTargetType
      ).alongWith(
        shooter.shootOnMoveCommandShooter(
          () -> drivetrain.getState().Pose,
          () -> ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds,
            drivetrain.getState().Pose.getRotation()),
          this::getShootTargetType
        ),
        Commands.waitUntil(() -> Math.abs(turret.getErrorDegrees()) <= Constants.Turret.kShootingToleranceDegrees)
          .andThen(new AutoUnjamCommand(spindexer, feeder))
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
      driveRobotCentric.withVelocityX(robotCentricDriveSpeed).withVelocityY(0)));
    joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
      driveRobotCentric.withVelocityX(0).withVelocityY(-robotCentricDriveSpeed)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
      driveRobotCentric.withVelocityX(-robotCentricDriveSpeed).withVelocityY(0)));
    joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
      driveRobotCentric.withVelocityX(0).withVelocityY(robotCentricDriveSpeed)));

    // Both bumpers together: stow intake
    joystick.rightBumper().and(joystick.leftBumper()).onTrue(
      intake.StowCommand().alongWith(
        Commands.runOnce(() -> {
          isShootOnMoveActive = false;
        })
      )
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
        if (isShootOnMoveActive && defenseDriveCommand.isScheduled()) {
            defenseDriveCommand.cancel();
        }
      })
    );

    // Derive Trigger objects from the flag — the scheduler reacts to these every loop
    Trigger shootOnMoveOn = new Trigger(() -> isShootOnMoveActive);
    Trigger triggerHeld = joystick.rightTrigger();
    Trigger inBlockedZone = new Trigger(() -> dashboard.getCurrentZone() == FieldZones.ZoneType.BLOCKED);
    
    Trigger warmUpActive = shootOnMoveOn.and(triggerHeld.or(inBlockedZone)); // ON + (held OR blocked)  → warm up
    Trigger firingActive = shootOnMoveOn.and(triggerHeld.negate().and(inBlockedZone.negate())); // ON + released + not blocked → fire

    // Warm-up phase: turret + flywheel only, both lead-compensated
    // Target type is resolved dynamically
    warmUpActive.whileTrue(
      turret.shootOnMoveCommandTurret(
        () -> drivetrain.getState().Pose,
        () -> ChassisSpeeds.fromRobotRelativeSpeeds(
          drivetrain.getState().Speeds,
          drivetrain.getState().Pose.getRotation()),
        this::getShootTargetType
      ).alongWith(
        shooter.shootOnMoveFlywheelOnlyCommand(
          () -> drivetrain.getState().Pose,
          () -> ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds,
            drivetrain.getState().Pose.getRotation()),
          this::getShootTargetType
        )
      ).withName("ShootOnMove-WarmUp")
    );

    // Firing phase: turret + full shooter (hood+flywheel) + spindexer + feeder, all lead-compensated
    // Target type is resolved dynamically
    firingActive.whileTrue(
      turret.shootOnMoveCommandTurret(
        () -> drivetrain.getState().Pose,
        () -> ChassisSpeeds.fromRobotRelativeSpeeds(
          drivetrain.getState().Speeds,
          drivetrain.getState().Pose.getRotation()),
        this::getShootTargetType
      ).alongWith(
        shooter.shootOnMoveCommandShooter(
          () -> drivetrain.getState().Pose,
          () -> ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds,
            drivetrain.getState().Pose.getRotation()),
          this::getShootTargetType
        ),
        Commands.waitUntil(() -> Math.abs(turret.getErrorDegrees()) <= Constants.Turret.kShootingToleranceDegrees)
          .andThen(new AutoUnjamCommand(spindexer, feeder))
      ).withName("ShootOnMove-Firing")
    );

    // When shoot-on-the-move mode is toggled OFF, move hood to trench position
    shootOnMoveOn.onFalse(
      shooter.setHoodToTrenchCommand()
    );

    // When shoot-on-the-move mode is toggled ON, intelligently deploy the intake
    shootOnMoveOn.onTrue(
      intake.ShootOnMoveDeployCommand()
    );

    // ── LEFT TRIGGER: Disable shoot on move, jiggle spindexer + intake ────────
    joystick.leftTrigger().onTrue(
      Commands.runOnce(() -> {
        isShootOnMoveActive = false;
        SmartDashboard.putBoolean("DASHBOARD/Shoot On Move Active", isShootOnMoveActive);
        if (defenseDriveCommand.isScheduled()) {
            defenseDriveCommand.cancel();
        }
      })
    );
    
    joystick.leftTrigger().whileTrue(
      spindexer.chillJiggleCommand().alongWith(new IntakeJiggleCommand(intake, -Constants.Intake.kSpinnerSpeed/3))
    );

    joystick.leftTrigger().onFalse(
      intake.DeployCommand()
    );

    //defualt shot
    joystick.y().onTrue(
      Commands.runOnce(() -> {
        isShootOnMoveActive = false;
        SmartDashboard.putBoolean("DASHBOARD/Shoot On Move Active", isShootOnMoveActive);
      })
    );
    joystick.y().whileTrue(new DefaultShootCommand(turret, shooter, () -> drivetrain.getState().Pose));
    joystick.y().whileTrue(
      spindexer.runCommand().alongWith(feeder.runCommand()));
    

    joystick.start().whileTrue(
      spindexer.reverseCommand().alongWith(feeder.reverseCommand())
    );

    // ── X BUTTON: Brake (lock wheels in X pattern while held) ───────────────
    joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));

    // ── right stick BUTTON: Toggle Defense Drive ─
    joystick.rightStick().toggleOnTrue(defenseDriveCommand);
    Trigger defenseModeActive = new Trigger(defenseDriveCommand::isScheduled);
    defenseModeActive.onTrue(Commands.runOnce(() -> {
      isShootOnMoveActive = false;
      SmartDashboard.putBoolean("DASHBOARD/Shoot On Move Active", isShootOnMoveActive);
    }));

    // Pathfind to nearest position in opponent zone while A+Y held, cancel on joystick move
    joystick.a().and(joystick.b()).onTrue(
      PathFindCommands.pathfindToNearestPose(
        () -> drivetrain.getState().Pose,
        List.of(FieldConstants.getLeftOpp(), FieldConstants.getRightOpp())
      ).until(joystick.leftStick().or(joystick.rightStick()))
    );

    // reset the field-centric heading on back button press(back button)
    joystick.back().onTrue(
      drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final
    var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
      drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    RobotModeTriggers.teleop().onTrue(
      shooter.setHoodToTrenchCommand()
    );

    new Trigger(() -> Math.round(GameState.timeRemainingInCurrentState()) == Constants.Misc.driverRumbleTime1).onTrue(rumble.lightPulse());
    new Trigger(() -> Math.round(GameState.timeRemainingInCurrentState()) == Constants.Misc.driverRumbleTime2).onTrue(rumble.doublePulse());

    new Trigger(() -> Math.round(GameState.timeRemainingInCurrentState()) == Constants.Misc.operatorRumbleTime1).onTrue(operatorRumble.lightPulse());
    new Trigger(() -> Math.round(GameState.timeRemainingInCurrentState()) == Constants.Misc.operatorRumbleTime2).onTrue(operatorRumble.lightPulse());
    new Trigger(() -> Math.round(GameState.timeRemainingInCurrentState()) == Constants.Misc.operatorRumbleTime3).onTrue(operatorRumble.doublePulse());

    //===============================================================================================================
    // Operator Controls
    //===============================================================================================================

    operatorJoystick.pov(0).onTrue(intake.DeployCommand());
    operatorJoystick.pov(90).onTrue(intake.StowCommand().alongWith(
        Commands.runOnce(() -> {
          isShootOnMoveActive = false;
        })
    ));
    operatorJoystick.pov(180).onTrue(intake.UltraStowCommand().alongWith(
        Commands.runOnce(() -> {
          isShootOnMoveActive = false;
        })
    ));
    operatorJoystick.pov(270).onTrue(intake.SpinnerStopCommand());

    operatorJoystick.rightBumper().whileTrue(intake.SpinnerMoveAtVelocityCommand(-9));

    operatorJoystick.b().whileTrue(spindexer.runCommand().alongWith(feeder.runCommand()));
    operatorJoystick.x().whileTrue(spindexer.reverseCommand().alongWith(feeder.reverseCommand()));

    operatorJoystick.a().onTrue(shooter.setHoodToTrenchCommand());
    operatorJoystick.y().whileTrue(new DefaultShootCommand(turret, shooter, () -> drivetrain.getState().Pose));

    
    
    //===============================================================================================================
    // Test controls 
    //===============================================================================================================



    testJoystick.a().and(testJoystick.pov(0)).whileTrue(
        shooter.setHoodAngleCommand(80)
    );
    testJoystick.a().and(testJoystick.pov(90)).whileTrue(
        shooter.setHoodAngleCommand(65)
    );
    testJoystick.a().and(testJoystick.pov(180)).whileTrue(
        shooter.setHoodAngleCommand(55)
    );
    testJoystick.a().and(testJoystick.pov(270)).whileTrue(
        shooter.setHoodAngleCommand(45)
    );

    // testJoystick.leftBumper().onTrue(shooter.flywheelTunableCommand(dashboard));
    testJoystick.leftBumper().whileTrue(
        shooter.shotTunableCommand(
            dashboard::getTunableSetpoint1,
            dashboard::getTunableSetpoint2
        )
    );

    testJoystick.b().and(testJoystick.pov(0)).whileTrue(
        shooter.runFlywheelsAtSpeedCommand(0)
    );
    testJoystick.b().and(testJoystick.pov(90)).whileTrue(
        shooter.runFlywheelsAtSpeedCommand(5)
    );
    testJoystick.b().and(testJoystick.pov(180)).whileTrue(
        shooter.runFlywheelsAtSpeedCommand(10)
    );
    testJoystick.b().and(testJoystick.pov(270)).whileTrue(
        shooter.runFlywheelsAtSpeedCommand(12)
    );

    //testJoystick.leftBumper().onTrue(turret.TurretTunableCommand(dashboard));

    testJoystick.rightBumper().and(testJoystick.pov(0)).whileTrue(
        turret.setAngleCommand(0)
    );
    testJoystick.rightBumper().and(testJoystick.pov(90)).whileTrue(
        turret.setAngleCommand(90)
    );
    testJoystick.rightBumper().and(testJoystick.pov(180)).whileTrue(
        turret.setAngleCommand(180)
    );
    testJoystick.rightBumper().and(testJoystick.pov(270)).whileTrue(
        turret.setAngleCommand(-90)
    );

    // //testJoystick.leftBumper().onTrue(feeder.tunableCommand(dashboard));

    testJoystick.x().and(testJoystick.pov(0)).whileTrue(
        feeder.runAtVelocityCommand(0)
    );
    testJoystick.x().and(testJoystick.pov(90)).whileTrue(
        feeder.runAtVelocityCommand(-10)
    );
    testJoystick.x().and(testJoystick.pov(180)).whileTrue(
        feeder.runAtVelocityCommand(-20)
    );
    testJoystick.x().and(testJoystick.pov(270)).whileTrue(
        feeder.runAtVelocityCommand(-30)
    );

    // //testJoystick.leftBumper().onTrue(spindexer.tunableCommand(dashboard));

    testJoystick.y().and(testJoystick.pov(0)).whileTrue(
        spindexer.runAtVelocityCommand(0)
    );
    testJoystick.y().and(testJoystick.pov(90)).whileTrue(
        spindexer.runAtVelocityCommand(1)
    );
    testJoystick.y().and(testJoystick.pov(180)).whileTrue(
        spindexer.runAtVelocityCommand(2)
    );
    testJoystick.y().and(testJoystick.pov(270)).whileTrue(
        spindexer.runAtVelocityCommand(3)
    );

 

    // testJoystick.leftBumper().onTrue(intake.PivotTunableCommand(dashboard));

    testJoystick.leftTrigger().and(testJoystick.pov(0)).whileTrue(
        intake.PivotSetAngleCommand(0
        )
    );
    testJoystick.leftTrigger().and(testJoystick.pov(90)).whileTrue(
        intake.PivotSetAngleCommand(45)
    );
    testJoystick.leftTrigger().and(testJoystick.pov(180)).whileTrue(
        intake.PivotSetAngleCommand(90)
    );
    testJoystick.leftTrigger().and(testJoystick.pov(270)).whileTrue(
        intake.PivotSetAngleCommand(135)
    );

    // //testJoystick.leftBumper().onTrue(intake.SpinnerTunableCommand(dashboard));

    testJoystick.rightTrigger().and(testJoystick.pov(0)).whileTrue(
        intake.SpinnerStopCommand()
    );
    testJoystick.rightTrigger().and(testJoystick.pov(90)).whileTrue(
        intake.SpinnerMoveAtVelocityCommand(-10)
    );
    testJoystick.rightTrigger().and(testJoystick.pov(180)).whileTrue(
        intake.SpinnerMoveAtVelocityCommand(-20)
    );
    testJoystick.rightTrigger().and(testJoystick.pov(270)).whileTrue(
        intake.SpinnerMoveAtVelocityCommand(-30)
    );

    // //testJoystick.leftBumper().onTrue(climber.tunableCommand(dashboard));

    // // Climber Test Controls
    // testJoystick.leftBumper().and(testJoystick.pov(0)).whileTrue(
    //     climber.upCommand()
    // );
    // testJoystick.leftBumper().and(testJoystick.pov(90)).whileTrue(
    //     climber.extendCommand()
    // );
    // testJoystick.leftBumper().and(testJoystick.pov(180)).whileTrue(
    //     climber.downCommand()
    // );
    // testJoystick.leftBumper().and(testJoystick.pov(270)).whileTrue(
    //     climber.retractCommand()
    // );

    testJoystick.back().whileTrue(feeder.runCommand());
    testJoystick.back().whileTrue(spindexer.runCommand());
    testJoystick.back().whileTrue(shooter.runFlywheelsAtSpeedCommand(4.5));

    testJoystick.start().whileTrue(feeder.runCommand());
    testJoystick.start().whileTrue(spindexer.runCommand());
    

    



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

    // joystick.a().whileTrue(spindexer.runCommand());
    // joystick.a().whileTrue(feeder.runCommand());

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

  /**
   * Returns the shoot TargetType to use based on the dashboard "Shoot Mode" chooser.
   * "Hub"  → TurretUtil.TargetType.HUB
   * "Pass" → whichever pass target (left/right) is nearest to the robot's current pose
   */
  private TurretUtil.TargetType getShootTargetType() {
    FieldZones.ZoneType currentZone = dashboard.getCurrentZone();
    if (currentZone == FieldZones.ZoneType.NEUTRAL) {
      return TurretUtil.getNearestPassTargetType(drivetrain.getState().Pose);
    } else if (currentZone == FieldZones.ZoneType.OPP) {
      return TurretUtil.getNearestLongPassTargetType(drivetrain.getState().Pose);
    }
    return TurretUtil.TargetType.HUB;
  }

  /**
   * Cycles the pathfind zone by {@code delta} steps (+1 = next, -1 = previous), wrapping around.
   * Publishes the new value to the dashboard.
   */
  private void cyclePathfindZone(int delta) {
    pathfindZoneIndex = Math.floorMod(pathfindZoneIndex + delta, PATHFIND_ZONES.length);
    dashboard.setPathfindZone(PATHFIND_ZONES[pathfindZoneIndex]);
  }

  /**
   * Cycles the shoot mode by {@code delta} steps (+1 = next, -1 = previous), wrapping around.
   * Publishes the new value to the dashboard.
   */
  private void cycleShootMode(int delta) {
    shootModeIndex = Math.floorMod(shootModeIndex + delta, SHOOT_MODES.length);
    dashboard.setShootMode(SHOOT_MODES[shootModeIndex]);
  }

  //** Called from Robot.java autonomousInit(), gets selected auto command */
  public Command getAutonomousCommand() {
    return dashboard.getAuto();
  }

  /** Called from Robot.java robotPeriodic(), updates dashboard */
  public void updateDashboard() {
    dashboard.update();
    // Update SYOMDrive status on SmartDashboard
    SmartDashboard.putBoolean("DASHBOARD/Defense Drive Enabled", defenseDriveCommand.isScheduled());
    // Update shoot-on-the-move mode toggle status on SmartDashboard
    SmartDashboard.putBoolean("DASHBOARD/Shoot On Move Active", isShootOnMoveActive);
    // Update swerve shoot-on-the-move mode toggle status on SmartDashboard
    SmartDashboard.putBoolean("DASHBOARD/Swerve Shoot On Move Active", isSwerveSOMActive);
    // Update whisker pickup mode toggle status on SmartDashboard
    SmartDashboard.putBoolean("DASHBOARD/Pickup Mode Active", isPickupModeActive);
  }

  /** Gets the current tunable value from the dashboard - use this for testing/tuning! */
  public double getTunableValue() {
    return dashboard.getTunableSetpoint1();
  }
}