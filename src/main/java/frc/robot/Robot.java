// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ProtoShooterSubsystem;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import frc.robot.commands.LimeLightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    TalonFX intake = new TalonFX(11); //Other names for this Rollers,Intake

    TalonFX arm = new TalonFX(13); //Other names for this Intake,Intake-Arm,Wrist,Expansion

    TalonFX Hoopper = new TalonFX(9); //Other names for this indexer,Hoopper,Basket,Feeder

    TalonFX Feeder = new TalonFX(8); // Other Names Jumper,Feeder,Accerlator

    ProtoShooterSubsystem shooter = new ProtoShooterSubsystem();

    TalonFX Turntable = new TalonFX(10);

    Joystick controller = new Joystick(0);

    Joystick controller2 = new Joystick(1);

    private Command m_autonomousCommand;

    private double targetRPM = 0;

    private double turretAngle = 0;

    private final RobotContainer m_robotContainer  = new RobotContainer();;
    private Field2d fieldPose = new Field2d();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        SmartDashboard.putData("robotpose", fieldPose);
    }

    @Override
    public void robotPeriodic() {
        fieldPose.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
        SmartDashboard.putData(fieldPose);

        SmartDashboard.putNumber("turretPOS", m_robotContainer.turret.getAngle().in(Degree));

        SwerveDriveState driveState = m_robotContainer.drivetrain.getState();

        m_timeAndJoystickReplay.update();
        // fieldPose.setRobotPose(llMeasurement.pose);
        SmartDashboard.putData("Robot Pose", fieldPose);
        CommandScheduler.getInstance().run();

        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
        // var llMeasurement = LimeLightHelpers.getBotPoseEstimate_wpiBlue(limelightName:"9600");
        var llMeasurement = LimeLightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-xflare"); //Our limelight's name is 9600

        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
            // fieldPose.setRobotPose(llMeasurement.pose);
            m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        }
        LimeLightHelpers.SetRobotOrientation("limelight-xflare", headingDeg, 0, 0, 0, 0, 0);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        SmartDashboard.putNumber("Target RPM", targetRPM);
        SmartDashboard.putNumber("turret angle", turretAngle);

    //     public void indicateLimelight(Indicate mode) {
    // Boolean b = (Math.floor(unitimer.get() * 10) % 2) == 1;
    // switch (mode) {
    //   case DISABLED:
    //     for (String limelight : Constants.Limelight.limelights) {
    //       LimelightHelpers.setLEDMode_ForceOff(limelight);
    //     }
    //     break;
    //   case ENABLED:
    //     for (String limelight : Constants.Limelight.limelights) {
    //       LimelightHelpers.setLEDMode_ForceOn(limelight);
    //     }
    //     break;
    //   case AUTO:
    //     for (String limelight : Constants.Limelight.limelights) {
    //       LimelightHelpers.setLEDMode_ForceBlink(limelight);
    //     }
    //     break;
//     }
//   }
  
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putNumber("Intake Arm", arm.getPosition().getValue().in(Rotations));
        SmartDashboard.putNumber("TurnTable", Turntable.getPosition().getValue().in(Rotations));
        // targetRPM = SmartDashboard.getNumber("Target RPM", targetRPM);
        // turretAngle = SmartDashboard.getNumber("turret angle", turretAngle);
        SmartDashboard.putNumber("Shooter RPM", shooter.getVelocity());
        SmartDashboard.putNumber("Intake POS", intake.getPosition().getValue().in(Degrees));

        if (controller.getRawButton(6)) {
            intake.set(-1);
            arm.setVoltage(3);
        }
        else if (controller.getRawAxis(3) > .4) {
            intake.set(-1);
            arm.setVoltage(-3);
        }
        else if (controller.getRawAxis(2) > .4) {
            intake.set(-1);
        }
        else if (controller.getRawButton(1)) {
            intake.set(1);
        }
        else {
            intake.set(0);
            arm.setVoltage(0);
        }

        // if (controller.getRawButton(4)) {
        //     shooter.runVelocity(-2680);
        // }
        // else if (!controller.getRawButton(4) || !controller.getRawButton(5)) {
        //     shooter.runVelocity(0;)
        // }


        
        // if (controller.getRawButton(1)) {
        //     m_robotContainer.turret.setAngleSetpoint(Degrees.of(turretAngle));
        // }
        // if (controller.getRawButton(3)) {
            // intake.set(-75);
        //     arm.setVoltage(-3);
        // }
        // else if(controller.getRawAxis(3) > .25) {
        //     arm.setPosition(0);
        // }
        // else if (controller.getRawButton(4)) {
        //     intake.set(-1);
        // }
        // else if (controller.getRawButton(7)) {
        //     intake.set(-1);
        //     arm.setVoltage(3);
        // }
        // else if (controller.getRawButton(6)) {
        //     Hoopper.setVoltage(-10);
        //     Feeder.setVoltage(10);
        // }
        // else if (controller.getRawButton(5)) {
        //     shooter.runVelocity(targetRPM);
        // }
        // else if (controller.getRawButton(8)) {
        //     shooter.runVelocity(0);
        // }
        // else {
        //     intake.set(0);
        //     arm.setVoltage(0);
        //     Hoopper.setVoltage(0);
        //     Feeder.setVoltage(0);
        //     Turntable.setVoltage(0);
        // }
        // if (controller2.getRawButton(3)) {
        //     if (Turntable.getPosition().getValue().in(Rotations) > -5) {
        //         Turntable.setVoltage(-1);
        //     }
        //     else {
        //         Turntable.setVoltage(0);
        //     }
        // }
        // else if (controller2.getRawButton(4)) {
        //     if (Turntable.getPosition().getValue().in(Rotations) < 5) {
        //         Turntable.setVoltage(1);
        //     }
        //     else {
        //         Turntable.setVoltage(0);
        //     }
        // }
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {}
       
    @Override
    public void testPeriodic() {

        SmartDashboard.putNumber("Turn", Turntable.getPosition().getValue().in(Rotations));

        // pos voltages in future make this a varable
        // outpost is close too 6.5
        // trench is 5.5
        // outpost/climb is 5 (not good with are hood angle)


         CommandScheduler.getInstance().cancelAll();

         if (controller.getRawButton(3)) {
            if (Turntable.getPosition().getValue().in(Rotations) > -5) {
                Turntable.setVoltage(-1);
            }
            else {
                Turntable.setVoltage(0);
            }
        }
        else if (controller.getRawButton(4)) {
            if (Turntable.getPosition().getValue().in(Rotations) < 5) {
                Turntable.setVoltage(1);
            }
            else {
                Turntable.setVoltage(0);
            }
        }
        else if (controller.getRawButton(6)) {
            if (Turntable.getPosition().getValue().in(Rotations) < 4 && Turntable.getPosition().getValue().in(Rotations) > 5) {
                Turntable.setVoltage(.5);
            }
            else if (Turntable.getPosition().getValue().in(Rotations) > 3.8 && Turntable.getPosition().getValue().in(Rotations) < 3) {
                Turntable.setVoltage(-.5);
            }
            else if (Turntable.getPosition().getValue().in(Rotations) > 3) {
                Turntable.setVoltage(-1);
            }
            else if (Turntable.getPosition().getValue().in(Rotations) > 5) {
                Turntable.setVoltage(-1);
            }
            else {
                Turntable.setVoltage(0);
            }
        }
        else if (controller.getRawButton(5)) {
            Turntable.setPosition(0);
        }
        else {
            Turntable.setVoltage(0);
            intake.set(0);
            arm.setVoltage(0);
            Hoopper.setVoltage(0);
            Feeder.setVoltage(0);
        }

        //reset turntable code

        // else if (controller.getRawButton(5)) {
        //     Turntable.setPosition(0);
        // }

        //------------------------------------

        // if (controller.getRawButton(3)) {
        //     intake.set(-.2);
        //     arm.setVoltage(.75);
        // }
        // else if (controller.getRawButton(1)) {
        //     intake.set(-.2);
        // }
        // else if (controller.getRawButton(2)) {
        //     intake.set(.2);
        //     arm.setVoltage(-.5);
        // }
        // else {
        //     intake.set(0);
        //     arm.setVoltage(0);
        // }
    }
    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
