// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ShootDistanceCommand;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.commands.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private boolean slowDownDriving = false;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public TurretSubsystem turret = new TurretSubsystem();

    private ShooterSubsystem shooter = new ShooterSubsystem();

    private HopperSubsystem hopper = new HopperSubsystem();

    public IntakeSubsystem intake = new IntakeSubsystem();

    public ShootOnTheMoveCommand shootCommand = new ShootOnTheMoveCommand(turret, shooter, hopper, drivetrain);

    private ShootDistanceCommand shootNoTurretCommand = new ShootDistanceCommand(turret, shooter, hopper, drivetrain);


    // private IntakeSubsystem deployAndRollCommand = new deployAndRollCommand();

    public RobotContainer() {
        
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
         autoChooser.addOption(
        "SysId Left Shooter Analysis",
        new SequentialCommandGroup(
            shooter.sysIdQuasistaticLeft(SysIdRoutine.Direction.kForward),
            shooter.sysIdQuasistaticLeft(SysIdRoutine.Direction.kReverse),
            shooter.sysIdDynamicLeft(SysIdRoutine.Direction.kForward),
            shooter.sysIdDynamicLeft(SysIdRoutine.Direction.kReverse)));

        NamedCommands.registerCommand("Drop command", intake.dropInatak());
        NamedCommands.registerCommand("RunInake command", intake.intakeCommand());
        NamedCommands.registerCommand("Shoot command", shootCommand);

        //put named command stuff here
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {

        joystick.b().whileTrue(Commands.runOnce(() -> slowDownDriving = true).andThen(shootCommand)).whileFalse(Commands.runOnce(() -> slowDownDriving = false));


        // joystick.b().whileTrue(shootNoTurretCommand);

        // joystick.x().whileTrue(ShooterSubsystem.setVelocitySetpoint(2000));

        joystick.x().whileTrue(intake.backFeedAndRollCommand());

        joystick.y().whileTrue(hopper.backFeedCommand());

        //if your going to add the intake command then please disable the simple code in robot.java

        // joystick.a().whileTrue(deployAndRollCommand);

        // joystick.b().whileTrue(hopper.feederRun());
        
        // joystick.y().whileTrue(turret.sysId());

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                // If shooting, drive at half speed
                double scale = slowDownDriving ? 0.5 : 1.0;

                double vx = -joystick.getLeftY() * MaxSpeed * scale;
                double vy = -joystick.getLeftX() * MaxSpeed * scale;
                double omega = -joystick.getRightX() * MaxAngularRate * scale;

                // if not driving, put wheels in X pattern
                boolean stopped =
                    Math.abs(vx) < MaxSpeed * 0.1 &&
                    Math.abs(vy) < MaxSpeed * 0.1 &&
                    Math.abs(omega) < MaxAngularRate * 0.1;

                if (stopped) {
                    return brake;
                }

                return drive.withVelocityX(vx)
                            .withVelocityY(vy)
                            .withRotationalRate(omega);
            })
        );


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.start().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
        // );
    }
}
