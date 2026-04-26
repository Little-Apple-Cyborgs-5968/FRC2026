package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class DefenseDriveCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Intake intake;
    private final Turret turret;
    private final DoubleSupplier velocityXSupplier;
    private final DoubleSupplier velocityYSupplier;
    private final DoubleSupplier rotationalRateSupplier;

    private final SwerveRequest.FieldCentric driveRequest;
    private final SwerveRequest.SwerveDriveBrake brakeRequest;

    private final double maxSpeed;
    private final double maxAngularRate;

    public DefenseDriveCommand(
            CommandSwerveDrivetrain drivetrain,
            Intake intake,
            Turret turret,
            DoubleSupplier velocityXSupplier,
            DoubleSupplier velocityYSupplier,
            DoubleSupplier rotationalRateSupplier,
            double maxSpeed,
            double maxAngularRate) {

        this.drivetrain = drivetrain;
        this.intake = intake;
        this.turret = turret;
        this.velocityXSupplier = velocityXSupplier;
        this.velocityYSupplier = velocityYSupplier;
        this.rotationalRateSupplier = rotationalRateSupplier;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;

        this.driveRequest = new SwerveRequest.FieldCentric()
                .withDeadband(maxSpeed * 0.1)
                .withRotationalDeadband(maxAngularRate * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        this.brakeRequest = new SwerveRequest.SwerveDriveBrake();

        addRequirements(drivetrain, intake, turret);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double vx = velocityXSupplier.getAsDouble();
        double vy = velocityYSupplier.getAsDouble();
        double omega = rotationalRateSupplier.getAsDouble();

        // Stow intake (ensure it stays stowed)
        intake.PivotSetAngle(Constants.Intake.kIntakeAngleStowed);

        // Snap turret to nearest 90 degrees
        double currentAngle = turret.getAngleDegrees();
        double targetAngle = Math.round(currentAngle / 90.0) * 90.0;
        turret.setAngle(targetAngle);

        boolean isStationary = Math.abs(vx) < (maxSpeed * 0.1) && Math.abs(vy) < (maxSpeed * 0.1) && Math.abs(omega) < (maxAngularRate * 0.1);

        if (isStationary && Constants.Swerve.kDefenseDriveXWheels) {
            drivetrain.setControl(brakeRequest);
        } else {
            drivetrain.setControl(
                    driveRequest
                            .withVelocityX(vx)
                            .withVelocityY(vy)
                            .withRotationalRate(omega));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
