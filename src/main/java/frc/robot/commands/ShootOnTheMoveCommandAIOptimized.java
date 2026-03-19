package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootOnTheMoveCommandAIOptimized extends Command {
  private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static final int LOOKAHEAD_ITERATIONS = 3;
  private static final boolean ENABLE_DEBUG = false;
  private static final int DEBUG_EVERY_N_LOOPS = 5;

  private static final double PHASE_DELAY_SECS = 0.05;

  private static final double ALLIANCE_MIN_DISTANCE_M = Units.inchesToMeters(104.0);
  private static final double ALLIANCE_MAX_DISTANCE_M = Units.inchesToMeters(241.0);
  private static final double PASS_MIN_DISTANCE_M = 0.75;
  private static final double PASS_MAX_DISTANCE_M = Units.inchesToMeters(500.0);

  private static final double TOF_MIN_IN = 54.33;
  private static final double TOF_MAX_IN = 223.62;

  static {
    // Inches -> RPM
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
    launchFlywheelSpeedMap.put(241.0, 3522.0); //calculated - replace with measured pair ASAP

    // Inches -> seconds
    timeOfFlightMap.put(223.62, 1.16);
    timeOfFlightMap.put(179.13, 1.12);
    timeOfFlightMap.put(124.02, 1.11);
    timeOfFlightMap.put(74.02, 1.09);
    timeOfFlightMap.put(54.33, 0.90);
  }

  private final TurretSubsystem turret;
  private final ShooterSubsystem shooterSubsystem;
  private final HopperSubsystem hopper;
  private final CommandSwerveDrivetrain swerveDrive;

  private final Debouncer shootingDebounce = new Debouncer(0.1, DebounceType.kFalling);
  private final Field2d debugField = new Field2d();

  private Rotation2d turretAngle = new Rotation2d();
  private int debugCounter = 0;

  public ShootOnTheMoveCommandAIOptimized(
      TurretSubsystem turret,
      ShooterSubsystem shooter,
      HopperSubsystem hopper,
      CommandSwerveDrivetrain swerveDrive) {

    this.turret = turret;
    this.shooterSubsystem = shooter;
    this.hopper = hopper;
    this.swerveDrive = swerveDrive;

    if (ENABLE_DEBUG) {
      SmartDashboard.putData("ShootOnTheMoveField", debugField);
    }

    addRequirements(turret, shooter, hopper);
  }

  @Override
  public void execute() {
    // Snapshot drivetrain state once
    var state = swerveDrive.getState();

    Pose2d robotPose =
        state.Pose.exp(
            new Twist2d(
                state.Speeds.vxMetersPerSecond * PHASE_DELAY_SECS,
                state.Speeds.vyMetersPerSecond * PHASE_DELAY_SECS,
                state.Speeds.omegaRadiansPerSecond * PHASE_DELAY_SECS));

    ChassisSpeeds fieldRelativeVelocity = swerveDrive.getFieldRelativeSpeed();

    boolean inAllianceZone = isInAllianceZone(robotPose);
    boolean onAllianceOutpostSide = !inAllianceZone && isOnAllianceOutpostSide(robotPose);

    double minDistanceM = inAllianceZone ? ALLIANCE_MIN_DISTANCE_M : PASS_MIN_DISTANCE_M;
    double maxDistanceM = inAllianceZone ? ALLIANCE_MAX_DISTANCE_M : PASS_MAX_DISTANCE_M;

    Translation2d target =
        inAllianceZone
            ? AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
            : onAllianceOutpostSide
                ? AllianceFlipUtil.apply(FieldConstants.Outpost.aimPoint)
                : AllianceFlipUtil.apply(FieldConstants.Depot.aimPoint);

    Pose2d turretPose = turret.getPose(robotPose);
    ChassisSpeeds turretVelocity =
        turret.getVelocity(fieldRelativeVelocity, robotPose.getRotation().getMeasure());

    double turretX = turretPose.getX();
    double turretY = turretPose.getY();
    double targetX = target.getX();
    double targetY = target.getY();

    // Initial distance
    double lookaheadX = turretX;
    double lookaheadY = turretY;
    double distanceM = Math.hypot(targetX - turretX, targetY - turretY);

    // 2-3 refinement passes is usually enough here
    double timeOfFlight = getTimeOfFlightSeconds(distanceM);
    for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
      lookaheadX = turretX + turretVelocity.vxMetersPerSecond * timeOfFlight;
      lookaheadY = turretY + turretVelocity.vyMetersPerSecond * timeOfFlight;
      distanceM = Math.hypot(targetX - lookaheadX, targetY - lookaheadY);
      timeOfFlight = getTimeOfFlightSeconds(distanceM);
    }

    double fieldAngleRad = Math.atan2(targetY - lookaheadY, targetX - lookaheadX);
    double robotRelativeTurretAngleRad = fieldAngleRad - robotPose.getRotation().getRadians();
    turretAngle = Rotation2d.fromRadians(MathUtil.angleModulus(robotRelativeTurretAngleRad));

    boolean publishDebug = ENABLE_DEBUG && ((debugCounter++ % DEBUG_EVERY_N_LOOPS) == 0);
    if (publishDebug) {
      debugField.setRobotPose(robotPose);
      SmartDashboard.putNumber("Turret Angle", turretAngle.getDegrees());
      SmartDashboard.putNumber("Distancetogoal", Units.metersToInches(distanceM));
    }

    if (distanceM < minDistanceM || distanceM > maxDistanceM) {
      hopper.stop();
      return;
    }

    // Uncomment this after you test performance
    double absTurretDeg = Math.abs(turretAngle.getDegrees());
    if (absTurretDeg > turret.MAX_ONE_DIR_FOV ) {
      if( !inAllianceZone ) {
        hopper.stop();
        shooterSubsystem.setDutyCycleSetpoint(0);
        return;
      }

      // Translation2d nearestHubCorner =
      //     turretAngle.getDegrees() > 0
      //         ? AllianceFlipUtil.apply(FieldConstants.Hub.rightFace.getTranslation())
      //         : AllianceFlipUtil.apply(FieldConstants.Hub.leftFace.getTranslation());

      // double dxHub = targetX - turretX;
      // double dyHub = targetY - turretY;
      // double dxCorner = nearestHubCorner.getX() - turretX;
      // double dyCorner = nearestHubCorner.getY() - turretY;

      // double hubDistSq = dxHub * dxHub + dyHub * dyHub;
      // double cornerDistSq = dxCorner * dxCorner + dyCorner * dyCorner;

      // // Protect against divide-by-zero if the turret is somehow on top of one of the points
      // if (hubDistSq > 1e-9 && cornerDistSq > 1e-9) {
      //   double halfHubWidth = FieldConstants.Hub.width * 0.5;
      //   double halfHubWidthSq = halfHubWidth * halfHubWidth;

      //   double cosAllowedExtraAngle =
      //       (hubDistSq + cornerDistSq - halfHubWidthSq)
      //           / (2.0 * Math.sqrt(hubDistSq * cornerDistSq));

      //   double excessFovDeg = absTurretDeg - turret.MAX_ONE_DIR_FOV;
      //   double cosExcessFov = Math.cos(Math.toRadians(excessFovDeg));

      //   // angleDeg < excessFovDeg  <=>  cos(angleDeg) > cos(excessFov)
      //   if (cosAllowedExtraAngle > cosExcessFov) {
          hopper.stop();
          return;
      //   }
      // }
    }

    // IMPORTANT: launch map is keyed in inches, not meters
    double distanceInches = Units.metersToInches(distanceM);
    double shooterRpmValue =
        inAllianceZone
            ? launchFlywheelSpeedMap.get(distanceInches)
            : passRpm(distanceInches);

    if (publishDebug) {
      SmartDashboard.putNumber("rpm", shooterRpmValue);
    }

    var shooterRpm = RPM.of(shooterRpmValue);

    turret.setAngleSetpoint(turretAngle.getMeasure());
    shooterSubsystem.setVelocitySetpoint(shooterRpm);

    boolean atSpeed =
        shootingDebounce.calculate(
            shooterSubsystem.getVelocity().isNear(shooterRpm, RPM.of(200)));

    if (atSpeed) {
      hopper.feed();
    }
  }

  private double getTimeOfFlightSeconds(double distanceMeters) {
    double distanceInches = MathUtil.clamp(Units.metersToInches(distanceMeters), TOF_MIN_IN, TOF_MAX_IN);
    return timeOfFlightMap.get(distanceInches);
  }

  private boolean isInAllianceZone(Pose2d robotPose) {
    double robotX = robotPose.getX();
    double allianceZoneX = AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone);
    return AllianceFlipUtil.shouldFlip() ? robotX > allianceZoneX : robotX < allianceZoneX;
  }

  private boolean isOnAllianceOutpostSide(Pose2d robotPose) {
    double robotY = robotPose.getY();
    double midLineY = AllianceFlipUtil.applyY(FieldConstants.LinesHorizontal.center);
    return AllianceFlipUtil.shouldFlip() ? robotY > midLineY : robotY < midLineY;
  }

  public double passRpm(double distanceInches) {
    return 7.538 * distanceInches + 1705.0;
  }

  /**
   * Returns the angle between sideA and sideB in radians.
   *
   * @param sideA one adjacent side
   * @param sideB the other adjacent side
   * @param oppositeSide the side opposite the angle
   * @return angle in radians
   */
  public static double angleRadians(double sideA, double sideB, double oppositeSide) {
      if (sideA <= 0 || sideB <= 0 || oppositeSide <= 0) {
          throw new IllegalArgumentException("All side lengths must be positive.");
      }

      double denominator = 2.0 * sideA * sideB;
      double cosTheta =
              (sideA * sideA + sideB * sideB - oppositeSide * oppositeSide) / denominator;

      // Clamp to protect against tiny floating-point errors
      cosTheta = Math.max(-1.0, Math.min(1.0, cosTheta));

      return Math.acos(cosTheta);
  }

  /**
   * Returns the angle between sideA and sideB in degrees.
   */
  public static double angleDegrees(double sideA, double sideB, double oppositeSide) {
      return Math.toDegrees(angleRadians(sideA, sideB, oppositeSide));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setDutyCycleSetpoint(0);
    hopper.stop();
  }
}