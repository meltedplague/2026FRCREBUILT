package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.NovaWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterSubsystem extends SubsystemBase
{
  private final TalonFX                    leaderMotor         = new TalonFX(15);
  private final TalonFX                    followerMotor         = new TalonFX(12);
  private final boolean                    followerInverted = true;
  private final SmartMotorControllerConfig motorConfig            = new SmartMotorControllerConfig(this)
      .withFollowers(Pair.of(followerMotor, followerInverted))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.34220, 0.11965, 0.0))
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private final SmartMotorController       motor                  = new TalonFXWrapper(leaderMotor,
                                                                                    DCMotor.getKrakenX60(2),
                                                                                    motorConfig);
  private final FlyWheelConfig             shooterConfig          = new FlyWheelConfig(motor)
      // Diameter of the flywheel.
      .withDiameter(Inches.of(4))
      // Mass of the flywheel.
      .withMass(Pounds.of(3.54))
      .withUpperSoftLimit(RPM.of(6065))
      .withLowerSoftLimit(RPM.of(0))
      .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);
  private final FlyWheel                   shooter                = new FlyWheel(shooterConfig);

  private final SysIdRoutine m_sysIdRoutineLeft =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> SignalLogger.writeString("Shooter/SysIdState/Left", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> motor.setVoltage(voltage), null, this));

  public ShooterSubsystem() {}

  /**
   * Gets the current velocity of the shooter.
   *
   * @return FlyWheel velocity.
   */
  public AngularVelocity getVelocity() {return shooter.getSpeed();}

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {return shooter.setSpeed(speed);}

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {return shooter.set(dutyCycle);}


  public Command setDutyCycle(Supplier<Double> dutyCycle) {return shooter.set(dutyCycle);}

  public Command setVelocity(Supplier<AngularVelocity> speed) {return shooter.run(speed);}

  @Override
  public void simulationPeriodic()
  {
    shooter.simIterate();
  }

  public Command sysIdOld() {
    return shooter.sysId(Volts.of(12), Volts.of(3).per(Second), Seconds.of(7));
  }

  @Override
  public void periodic()
  {
    shooter.updateTelemetry();
  }

  public void setRPM(LinearVelocity newHorizontalSpeed)
  {
    shooter.setMeasurementVelocitySetpoint(newHorizontalSpeed);
  }

  public boolean readyToShoot(AngularVelocity tolerance)
  {
    if (motor.getMechanismSetpointVelocity().isEmpty())
    {return false;}
    return motor.getMechanismVelocity().isNear(motor.getMechanismSetpointVelocity().orElseThrow(), tolerance);
  }

  public void setVelocitySetpoint(AngularVelocity speed)
  {
    shooter.setMechanismVelocitySetpoint(speed);
  }

  public void setDutyCycleSetpoint(double dutyCycle)
  {
    shooter.setDutyCycleSetpoint(dutyCycle);
  }

  /**
   * Full YAMS SysId routine wrapped with CTRE SignalLogger start/stop.
   *
   * Tune these values if needed:
   * - dynamic step voltage: 12 V
   * - ramp rate: 3 V/s
   * - timeout: 7 s
   */
  public Command sysId() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          SignalLogger.stop();   // make sure any old log is closed
          SignalLogger.start();  // start fresh log for this run
        }),
        shooter.sysId(Volts.of(12), Volts.of(3).per(Second), Seconds.of(7))
    ).finallyDo(interrupted -> {
      shooter.setDutyCycleSetpoint(0.0);
      SignalLogger.stop();
    });
  }
   public Command sysIdQuasistaticLeft(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLeft.quasistatic(direction);
  }

  public Command sysIdDynamicLeft(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLeft.dynamic(direction);
}
}