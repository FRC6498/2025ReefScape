// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.LiftConstants;

public class Lift extends SubsystemBase {
  private final TalonFX leftMotor, rightMotor;

  // sysid
  // measures for the left motor
  private final MutVoltage appliedVoltageLeft = Volts.mutable(0);
  private final MutAngle liftDisplacementLeft = Radians.mutable(0);
  private final MutAngularVelocity liftVelocityLeft = RadiansPerSecond.mutable(0);
  private final MutAngularAcceleration liftAccelerationLeft = RadiansPerSecondPerSecond.mutable(0);
  // measures for the right motor
  private final MutVoltage appliedVoltageRight = Volts.mutable(0);
  private final MutAngle liftDisplacementRight = Radians.mutable(0);
  private final MutAngularVelocity liftVelocityRight = RadiansPerSecond.mutable(0);
  private final MutAngularAcceleration liftAccelerationRight = RadiansPerSecondPerSecond.mutable(0);
  private final SysIdRoutine routine;

  public Lift() {
    // configure motors

    leftMotor = new TalonFX(LiftConstants.LEFT_LIFT_MOTOR_ID);
    rightMotor = new TalonFX(LiftConstants.RIGHT_LIFT_MOTOR_ID);
    rightMotor.get();
    leftMotor.get();
    // slave the left side to the right side to prevent them getting out of sync
    leftMotor.setControl(new Follower(LiftConstants.RIGHT_LIFT_MOTOR_ID, false));

    // Configure Motors (should not affect sysid)
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0 = Constants.LiftConstants.LIFT_MOTOR_CONFIG;
    config.MotionMagic = Constants.LiftConstants.LIFT_MOTION_CONFIGS;
    rightMotor.getConfigurator().apply(config);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
    leftMotor.getConfigurator().apply(config);
    leftMotor.setNeutralMode(NeutralModeValue.Brake);


    // configure Sysid
    // NOTE::
    // anything with variable -> {*some other stuff*}
    // is called a consumer. These are functions that do something but return
    // nothing.
    // In sysid, all the functions do is either set motor voltages or write data to
    // logs
    // so consumers make things more efficient

    routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            voltage -> {
              // set the voltage of the two motors
              leftMotor.setVoltage(voltage.magnitude());
              rightMotor.setVoltage(voltage.magnitude());
            }, log -> {
              // see the elevator identification section of the wpi docs for an explanation of
              // why these parameters need to be logged
              log.motor("LiftMotorLeft")
                  .voltage(appliedVoltageLeft.mut_replace(leftMotor.getMotorVoltage().getValue()))
                  .angularAcceleration(liftAccelerationLeft.mut_replace(leftMotor.getAcceleration().getValue()))
                  .angularVelocity(liftVelocityLeft.mut_replace(leftMotor.getVelocity().getValue()))
                  .angularPosition(liftDisplacementLeft.mut_replace(leftMotor.getPosition().getValue()));
              log.motor("LiftMotorRight")
                  .voltage(appliedVoltageRight.mut_replace(rightMotor.getMotorVoltage().getValue()))
                  .angularAcceleration(liftAccelerationRight.mut_replace(rightMotor.getAcceleration().getValue()))
                  .angularVelocity(liftVelocityRight.mut_replace(rightMotor.getVelocity().getValue()))
                  .angularPosition(liftDisplacementRight.mut_replace(rightMotor.getPosition().getValue()));
            }, this));
  }

  /**
   * Dynamic Sysid test for the lift - will run the motors at at 7v (~60%) power
   * until the test is stopped
   * 
   * @param direction
   * @return Command
   */
  public Command liftSysidDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  /**
   * Quasistatic Sysid test for the lift - will run the motors at an increasing
   * voltage up to 7v at a 0.5V step
   * 
   * @param direction
   * @return
   */
  public Command liftSysidQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command runToRotations(double rotations) {
    MotionMagicVoltage requestRight = new MotionMagicVoltage(rightMotor.getPosition().getValueAsDouble());
    MotionMagicVoltage requestLeft = new MotionMagicVoltage(leftMotor.getPosition().getValueAsDouble());
    return runOnce(() -> {
      rightMotor.setControl(requestRight.withPosition(rotations));
      leftMotor.setControl(requestLeft.withPosition(rotations));
    });
  }

  public Command runToRotations(DoubleSupplier rotations) {
    return runToRotations(rotations.getAsDouble());
  }

  public Command zeroLift(){
    return runOnce(()->{
      rightMotor.setPosition(0);
      leftMotor.setPosition(0);
    });
  }

  public double getRotations(){
    return leftMotor.getPosition().getValueAsDouble();
  }

  public Command scrimageSetup(double speed) {
    return runOnce(() -> {
      leftMotor.set(speed);
      rightMotor.set(speed);
    });
  }

  public Command liftStop() {
    return runOnce(() -> {
      leftMotor.stopMotor();
      rightMotor.stopMotor();
    });
  }

  public Command liftHold() {
    return runOnce(() -> {
      leftMotor.setVoltage(.5);
      rightMotor.setVoltage(.5);
    });
  }

  public Command hardStopAll_DANGER() {
    return runOnce(() -> {
      CommandScheduler.getInstance().cancelAll(); // this will
      leftMotor.setVoltage(0);
      rightMotor.setVoltage(0);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("lift rotations", getRotations());
    SmartDashboard.putNumber("delta rot", getRotations() - rightMotor.getPosition().getValueAsDouble());
  }
}
