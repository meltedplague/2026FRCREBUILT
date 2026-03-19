package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase
{
public final double MAX_ONE_DIR_FOV = 90; // degrees
  // Stay away from the ends during SysId
private static final double SYSID_SAFE_MIN_DEG = -60.0;
private static final double SYSID_SAFE_MAX_DEG = 60.0;

// Conservative profile for first bring-up
private static final double CRUISE_VEL_DEG_PER_SEC = 2440.0;
private static final double ACCEL_DEG_PER_SEC_SQ = 2440.0;
  public final Translation3d turretTranslation = new Translation3d(0.205, 0, 0.375);
  private final TalonFX                   turretMotor       = new TalonFX(10);//, MotorType.kBrushless);
  private final SmartMotorControllerConfig motorConfig      = new SmartMotorControllerConfig(this)
      .withClosedLoopController(40, 0, 0, DegreesPerSecond.of(CRUISE_VEL_DEG_PER_SEC), DegreesPerSecondPerSecond.of(ACCEL_DEG_PER_SEC_SQ)) //TODO You need to tune kP
      .withSoftLimit(Degrees.of(-MAX_ONE_DIR_FOV), Degrees.of(MAX_ONE_DIR_FOV))
      .withGearing(new MechanismGearing(30.0))
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(25))
      .withMotorInverted(true)
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1))
      .withFeedforward(new SimpleMotorFeedforward(/* old 0.14851*/.24, /*3.3577*/7.5, 0.17251))
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController       motor            = new TalonFXWrapper(turretMotor,
                                                                                  DCMotor.getKrakenX60(1),
                                                                                  motorConfig);

                                                                                  private final VoltageOut sysIdVoltageRequest = new VoltageOut(0.0);

private final SysIdRoutine turretSysIdRoutine =
new SysIdRoutine(
new SysIdRoutine.Config(
Volts.of(0.35).per(Second),
Volts.of(1.5),
Seconds.of(3.0),
state -> SignalLogger.writeString("state", state.toString())
),
new SysIdRoutine.Mechanism(
volts -> turretMotor.setControl(
sysIdVoltageRequest.withOutput(volts.in(Volts))
),
null,
this,
"turret"
)
);

  // TODO set your correct offset
  private final MechanismPositionConfig    robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(0.762))
      .withMaxRobotLength(Meters.of(0.75))
      .withMovementPlane(Plane.XY)
      .withRelativePosition(turretTranslation); 
  private final PivotConfig                m_config         = new PivotConfig(motor)
      .withHardLimit(Degrees.of(-MAX_ONE_DIR_FOV - 5), Degrees.of(MAX_ONE_DIR_FOV + 5))
      .withTelemetry("Turret", TelemetryVerbosity.HIGH)
      .withStartingPosition(Degrees.of(0))
      .withMOI(0.05)
      .withMechanismPositionConfig(robotToMechanism);
  private final Pivot                      turret           = new Pivot(m_config);

  // Robot to turret transform, from center of robot to turret.
  private final Transform3d roboToTurret = new Transform3d(Feet.of(.67257), Feet.of(0), Feet.of(1.230315), Rotation3d.kZero); //likely -0.205, 0.0, 0.375, 0.0

  public TurretSubsystem()
  // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
  {
    //       in the constructor or in the robot coordination class, such as RobotContainer.
    //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
    //       such as SpeedControllers, Encoders, DigitalInputs, etc.
  }

  public Pose2d getPose(Pose2d robotPose)
  {
    return robotPose.plus(new Transform2d(
        roboToTurret.getTranslation().toTranslation2d(), roboToTurret.getRotation().toRotation2d()));
  }

  public ChassisSpeeds getVelocity(ChassisSpeeds robotVelocity, Angle robotAngle)
  {
    var robotAngleRads = robotAngle.in(Radians);
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
        + robotVelocity.omegaRadiansPerSecond
          * (roboToTurret.getY() * Math.cos(robotAngleRads)
             - roboToTurret.getX() * Math.sin(robotAngleRads));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
        + robotVelocity.omegaRadiansPerSecond
          * (roboToTurret.getX() * Math.cos(robotAngleRads)
             - roboToTurret.getY() * Math.sin(robotAngleRads));

    return new ChassisSpeeds(turretVelocityX,
                             turretVelocityY,
                             robotVelocity.omegaRadiansPerSecond + motor.getMechanismVelocity().in(RadiansPerSecond));
  }

  public void periodic()
  {
    turret.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    turret.simIterate();
  }

  public Command turretCmd(double dutycycle)
  {
    return turret.set(dutycycle);
  }

  public Command sysIdOLD()
  {
    return turret.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }

  public Command setAngle(Angle angle)
  {
    return turret.setAngle(angle);
  }

  public void setAngleSetpoint(Angle measure)
  {
    turret.setMechanismPositionSetpoint(measure);
  }
  public Angle getAngle() {
    return turret.getAngle();
  }
  public boolean atAngle(Angle target, double toleranceDeg) {
return Math.abs(getAngle().in(Degrees) - target.in(Degrees)) <= toleranceDeg
&& Math.abs(motor.getMechanismVelocity().in(DegreesPerSecond)) <= 15.0;
}

public Command moveToCenter() {
return Commands.runOnce(() -> setAngleSetpoint(Degrees.of(0)), this)
.andThen(Commands.waitUntil(() -> atAngle(Degrees.of(0), 2.0)));
}

public boolean inSysIdSafeWindow() {
double deg = getAngle().in(Degrees);
return deg >= SYSID_SAFE_MIN_DEG && deg <= SYSID_SAFE_MAX_DEG;
}

public Command sysId() {
return moveToCenter()
.andThen(Commands.waitSeconds(0.25))
.andThen(
turret.sysId(
Volts.of(1.5), // dynamic step voltage
Volts.of(0.35).per(Second), // quasistatic ramp rate
Seconds.of(3.0))); // timeout
}
public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  return turretSysIdRoutine.quasistatic(direction);
}

public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  return turretSysIdRoutine.dynamic(direction);
}
}