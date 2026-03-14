package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

public class HopperSubsystem extends SubsystemBase {

  private static final double HOPPER_SPEED = 1.0;
  private static final double FEEDER_SPEED = 1.0;

  private TalonFX hopperMotor = new TalonFX(9);
  private final TalonFX                    feederMotor            = new TalonFX(8);
  
  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("HopperMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(4))) // 4:1 gear reduction
      .withMotorInverted(true)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController smc = new TalonFXWrapper(hopperMotor, DCMotor.getKrakenX60(1), smcConfig);

  private final FlyWheelConfig hopperConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("Hopper", TelemetryVerbosity.HIGH);

  private FlyWheel hopper = new FlyWheel(hopperConfig);

  private SmartMotorControllerConfig smcFeederConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(4))) // 4:1 gear reduction
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(20));

  private SmartMotorController smcFeeder = new TalonFXWrapper(feederMotor, DCMotor.getKrakenX60(1), smcFeederConfig);

  private final FlyWheelConfig feederConfig = new FlyWheelConfig(smcFeeder)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("Feeder", TelemetryVerbosity.HIGH);

  private FlyWheel feeder = new FlyWheel(feederConfig);

  public HopperSubsystem() {
  }

  public void feed() {
    hopper.setDutyCycleSetpoint(HOPPER_SPEED);
    feeder.setDutyCycleSetpoint(FEEDER_SPEED);
  }
  public Command feederRun() {
    return feeder.set(FEEDER_SPEED);
  }

  public void stop() {
    hopper.setDutyCycleSetpoint(0.0);
    feeder.setDutyCycleSetpoint(0.0);
  }

  /**
   * Command to run the hopper forward while held.
   */
  public Command feedCommand() {
    return hopper.set(HOPPER_SPEED).alongWith(feeder.set(FEEDER_SPEED)).finallyDo(() -> {smc.setDutyCycle(0); smcFeeder.setDutyCycle(0);}).withName("Hopper.Feed");
  }

  public Command backFeedCommand() {
    return hopper.set(-HOPPER_SPEED).alongWith(feeder.set(-FEEDER_SPEED)).finallyDo(() -> {smc.setDutyCycle(0); smcFeeder.setDutyCycle(0);}).withName("Hopper.BackFeed");
  }

  /**
   * Command to run the hopper in reverse while held.
   */
  public Command reverseCommand() {
    return hopper.set(-HOPPER_SPEED).alongWith(feeder.set(-FEEDER_SPEED)).finallyDo(() -> {smc.setDutyCycle(0); smcFeeder.setDutyCycle(0);}).withName("Hopper.Reverse");
  }

  /**
   * Command to stop the hopper.
   */
  public Command stopCommand() {
    return hopper.set(0).alongWith(feeder.set(0)).withName("Hopper.Stop");
  }

  @Override
  public void periodic() {
    hopper.updateTelemetry();
    feeder.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    hopper.simIterate();
    feeder.simIterate();
  }
}
