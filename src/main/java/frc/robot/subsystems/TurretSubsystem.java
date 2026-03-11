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

import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private final double MAX_ONE_DIR_FOV = 90; // degrees
  public final Translation3d turretTranslation = new Translation3d(-0.205, 0.0, 0.375);
  private final TalonFX                   turretMotor       = new TalonFX(10);//, MotorType.kBrushless);
  private final SmartMotorControllerConfig motorConfig      = new SmartMotorControllerConfig(this)
      .withClosedLoopController(15.0, 0, 0, DegreesPerSecond.of(2440), DegreesPerSecondPerSecond.of(2440)) //TODO You need to tune kP
      .withSoftLimit(Degrees.of(-MAX_ONE_DIR_FOV), Degrees.of(MAX_ONE_DIR_FOV))
      .withGearing(new MechanismGearing(30.0))
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(10))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1))
      .withFeedforward(new SimpleMotorFeedforward(0, 7.5, 0))
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController       motor            = new TalonFXWrapper(turretMotor,
                                                                                  DCMotor.getKrakenX60(1),
                                                                                  motorConfig);
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
  private final Transform3d roboToTurret = new Transform3d(Feet.of(-1.5), Feet.of(0), Feet.of(0.5), Rotation3d.kZero); //likely -0.205, 0.0, 0.375, 0.0

  public TurretSubsystem()
  {
    // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
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

  public Command sysId()
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
}