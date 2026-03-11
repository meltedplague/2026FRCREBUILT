package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.Supplier;
import yams.mechanisms.swerve.SwerveDrive;


/**
 * Adapted from 6328 Mechanical Advantage!
 * Original source is here: https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java
 */
public class ShootOnTheMoveCommand extends Command
{

  private final double     loopPeriodSecs = Milliseconds.of(20).in(Seconds);
  // Outputs
  private       Rotation2d lastTurretAngle;
  private       Rotation2d turretAngle;

  // Private Variables
  private              TurretSubsystem                          turret;
  private              ShooterSubsystem                         shooterSubsystem;
  private              HopperSubsystem                          hopper;
  private              Supplier<ChassisSpeeds>                  _fieldRelativeVelocity;
  private              Supplier<Pose2d>                         estimatedPose;
  private              Field2d                                  debugField             = new Field2d();
  private static final InterpolatingDoubleTreeMap               launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap               timeOfFlightMap        =
      new InterpolatingDoubleTreeMap();

  // Tuning Constants
  private final Debouncer shootingDebounce = new Debouncer(0.1, DebounceType.kFalling);
  private final double    phaseDelay       = 0.05;


  static
  {
    // These should be found on your robot
    launchFlywheelSpeedMap.put(184.0, 3100.0);
    launchFlywheelSpeedMap.put(173.0, 3000.0);
    launchFlywheelSpeedMap.put(131.5, 2700.0);
    launchFlywheelSpeedMap.put(111.0, 2650.0);
    launchFlywheelSpeedMap.put(104.0, 2600.0);
    launchFlywheelSpeedMap.put(105.0, 2600.0);
    launchFlywheelSpeedMap.put(122.0, 2680.0);
    launchFlywheelSpeedMap.put(161.0, 2900.0);
    launchFlywheelSpeedMap.put(142.0, 2760.0);
    launchFlywheelSpeedMap.put(158.5, 2840.0);
    launchFlywheelSpeedMap.put(192.0, 3150.0);
    launchFlywheelSpeedMap.put(162.0, 2930.0);
    launchFlywheelSpeedMap.put(151.0, 2800.0);

    // TODO You likely need to measure this
    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  public ShootOnTheMoveCommand(TurretSubsystem turret, ShooterSubsystem shooter, HopperSubsystem hopper,
                               CommandSwerveDrivetrain swerveDrive)
  {
    SmartDashboard.putData("ShootOnTheMoveField", debugField);
    estimatedPose = () -> {
      // Calculate estimated pose while accounting for phase delay
      ChassisSpeeds robotRelativeVelocity = swerveDrive.getState().Speeds;
      var           robotPose             = swerveDrive.getState().Pose;
      robotPose = robotPose.exp(
          new Twist2d(
              robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
              robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
              robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));
      // Optional, add logging here
      debugField.setRobotPose(robotPose);
      return robotPose;
    };
    _fieldRelativeVelocity = swerveDrive::getFieldRelativeSpeed;

    addRequirements(turret, shooter, hopper);

  }

  @Override
  public void initialize()
  {

  }

  @Override
  public void execute()
  {
    // Get estimated pose
    var robotPose             = estimatedPose.get();
    var fieldRelativeVelocity = _fieldRelativeVelocity.get();
    Distance  minDistance      = isInAllianceZone(robotPose) ? Inches.of(104) : Meters.of(0.75);
    Distance  maxDistance      = isInAllianceZone(robotPose) ? Inches.of(192) : Inches.of(500);

    // Calculate distance from turret to target
    Translation2d target = isInAllianceZone(robotPose) ?
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()) :
        isOnAllianceOutpostSide(robotPose) ?
        AllianceFlipUtil.apply(FieldConstants.Outpost.aimPoint) :
        AllianceFlipUtil.apply(FieldConstants.Depot.aimPoint);
    Pose2d turretPosition         = turret.getPose(robotPose);
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    Angle         robotAngle     = robotPose.getRotation().getMeasure();
    ChassisSpeeds turretVelocity = turret.getVelocity(fieldRelativeVelocity, robotAngle);

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose                   = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++)
    {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocity.vxMetersPerSecond * timeOfFlight;
      double offsetY = turretVelocity.vyMetersPerSecond * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    if (lastTurretAngle == null) {lastTurretAngle = turretAngle;}
    lastTurretAngle = turretAngle;
    var lookaheadTurretToTargetDistanceMeasure = Meters.of(lookaheadTurretToTargetDistance);
    if (lookaheadTurretToTargetDistanceMeasure.gte(minDistance) &&
        lookaheadTurretToTargetDistanceMeasure.lte(maxDistance))
    {
      var shooterRPM = isInAllianceZone(robotPose) ? RPM.of(launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance)) : RPM.of(passRpm(lookaheadTurretToTargetDistanceMeasure.in(Inches)));
      turret.setAngleSetpoint(turretAngle.getMeasure());
      shooterSubsystem.setVelocitySetpoint(shooterRPM);
      if (shootingDebounce.calculate(shooterSubsystem.getVelocity().isNear(shooterRPM, RPM.of(10))))
      {
        hopper.feed();
      } else {
        hopper.stop();
      }
    } else {
      hopper.stop();
    }

  }

  private boolean isInAllianceZone(Pose2d robotPose) {
    Distance robotXDistance = robotPose.getMeasureX();
    double allianceZoneMeters = AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone);
    return AllianceFlipUtil.shouldFlip() ? robotXDistance.gt(Meters.of(allianceZoneMeters)) : robotXDistance.lt(Meters.of(allianceZoneMeters));
  }

  private boolean isOnAllianceOutpostSide(Pose2d robotPose) {
    Distance robotYDistance = robotPose.getMeasureY();
    double midLineMeters = AllianceFlipUtil.applyY(FieldConstants.LinesHorizontal.center);

    return AllianceFlipUtil.shouldFlip() ? robotYDistance.gt(Meters.of(midLineMeters)) : robotYDistance.lt(Meters.of(midLineMeters));
  }

  public double passRpm(double distanceInches) {
    return 7.538 * distanceInches + 1705.0;
  }

  @Override
  public boolean isFinished()
  {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    shooterSubsystem.setDutyCycleSetpoint(0);
    hopper.stop();
  }
}