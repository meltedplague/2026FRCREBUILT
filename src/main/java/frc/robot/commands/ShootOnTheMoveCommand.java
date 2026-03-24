package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
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
  private final double     HUB_RPM_ADJUSTMENT = 50.0;
  private final double     PASS_RPM_ADJUSTMENT = 50.0;
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
    //                         Inches |  RPMS 
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
    launchFlywheelSpeedMap.put(241.0, 3522.0); //calculated - replace with measured asap

    //                  Inches    | Seconds in the air
    timeOfFlightMap.put(172.0, 0.99);
    timeOfFlightMap.put(147.0, 0.90);
    timeOfFlightMap.put(128.0, 0.83);
    timeOfFlightMap.put(127.0, 0.83);
    timeOfFlightMap.put(112.0, 0.80);
    timeOfFlightMap.put(241.0, 1.21); //calculated - replace with measured asap
    timeOfFlightMap.put(104.0, 0.76); //calculated - replace with measured asap
  }

  public ShootOnTheMoveCommand(TurretSubsystem turret, ShooterSubsystem shooter, HopperSubsystem hopper,
                               CommandSwerveDrivetrain swerveDrive)
  {
    this.turret = turret;
    this.shooterSubsystem = shooter;
    this.hopper = hopper;

  
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
    boolean inAllianceZone    = isInAllianceZone(robotPose);
    var fieldRelativeVelocity = _fieldRelativeVelocity.get();
    Distance  minDistance      = inAllianceZone  ? Inches.of(104) : Meters.of(0.75);
    Distance  maxDistance      = inAllianceZone  ? Inches.of(241) : Inches.of(500);

    // Calculate distance from turret to target
    Translation2d target = inAllianceZone  ?
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
      timeOfFlight = inAllianceZone  ? timeOfFlightMap.get(Meters.of(lookaheadTurretToTargetDistance).in(Inches)) : passTOF(Meters.of(lookaheadTurretToTargetDistance).in(Inches));
      double offsetX = turretVelocity.vxMetersPerSecond * timeOfFlight;
      double offsetY = turretVelocity.vyMetersPerSecond * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    // turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    Rotation2d fieldangletotarget = target.minus(lookaheadPose.getTranslation()).getAngle();
    Rotation2d robotrelativeturretangle = fieldangletotarget.minus(robotPose.getRotation());
    turretAngle =Rotation2d.fromRadians(MathUtil.angleModulus(robotrelativeturretangle.getRadians()));
    SmartDashboard.putNumber("Turret Angle", turretAngle.getDegrees());
    if (lastTurretAngle == null) {lastTurretAngle = turretAngle;}
    lastTurretAngle = turretAngle;
    var lookaheadTurretToTargetDistanceMeasure = Meters.of(lookaheadTurretToTargetDistance);
    SmartDashboard.putNumber("Distancetogoal", lookaheadTurretToTargetDistanceMeasure.in(Inches));
    
    if (!(lookaheadTurretToTargetDistanceMeasure.gte(minDistance) &&
        lookaheadTurretToTargetDistanceMeasure.lte(maxDistance)))
    {
      hopper.stop();
      return;
    }

    double absTurretDeg = Math.abs(turretAngle.getDegrees());
    if (absTurretDeg > turret.MAX_ONE_DIR_FOV ) {
      if( !inAllianceZone ) {
        hopper.stop();
        shooterSubsystem.setDutyCycleSetpoint(0);
        return;
      }

      Translation2d nearestHubCorner =
          turretAngle.getDegrees() > 0
              ? AllianceFlipUtil.apply(FieldConstants.Hub.rightFace.getTranslation())
              : AllianceFlipUtil.apply(FieldConstants.Hub.leftFace.getTranslation());

      double turretToHubCornerDistance = nearestHubCorner.getDistance(turretPosition.getTranslation());

      double hubDistSq = turretToTargetDistance * turretToTargetDistance;
      double cornerDistSq = turretToHubCornerDistance * turretToHubCornerDistance;

      double halfHubWidth = FieldConstants.Hub.width * 0.5;
      double halfHubWidthSq = halfHubWidth * halfHubWidth;

      double cosAllowedExtraAngle =
          (hubDistSq + cornerDistSq - halfHubWidthSq)
              / (2.0 * Math.sqrt(hubDistSq * cornerDistSq));

      double excessFovDeg = absTurretDeg - turret.MAX_ONE_DIR_FOV;
      double cosExcessFov = Math.cos(Math.toRadians(excessFovDeg));

      // angleDeg < excessFovDeg  <=>  cos(angleDeg) > cos(excessFov)
      if (cosAllowedExtraAngle > cosExcessFov) {
        hopper.stop();
        return;
      }
    }

    var shooterRPM = inAllianceZone  ? RPM.of(launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance) + HUB_RPM_ADJUSTMENT) : RPM.of(passRpm(lookaheadTurretToTargetDistanceMeasure.in(Inches) + PASS_RPM_ADJUSTMENT));
    SmartDashboard.putNumber("rpm",shooterRPM.in(RPM));
    turret.setAngleSetpoint(turretAngle.getMeasure());
    shooterSubsystem.setVelocitySetpoint(shooterRPM);
    if (shootingDebounce.calculate(shooterSubsystem.getVelocity().isNear(shooterRPM, RPM.of(200)))) // If you have problems with this you increase this by 10 rpms each time until it shoots like you want. If you start creeping above 100, then you might want to look at something else as the problem.
    {
      hopper.feed();
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

  public double passTOF(double distanceInches) {
    return 0.00328984 * distanceInches + 0.418633;
  }

  public void diagnosticInfo() {
    var robotPose             = estimatedPose.get();
    Translation2d target = isInAllianceZone(robotPose) ?
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()) :
        isOnAllianceOutpostSide(robotPose) ?
        AllianceFlipUtil.apply(FieldConstants.Outpost.aimPoint) :
        AllianceFlipUtil.apply(FieldConstants.Depot.aimPoint);
    Pose2d turretPosition         = turret.getPose(robotPose);
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    var lookaheadTurretToTargetDistanceMeasure = Meters.of(lookaheadTurretToTargetDistance);
    SmartDashboard.putNumber("Turret Angle", turret.getAngle().in(Degrees));
    SmartDashboard.putNumber("Distancetogoal", lookaheadTurretToTargetDistanceMeasure.in(Inches));
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
    turret.turretSetDutyCycle(0);
  }
}