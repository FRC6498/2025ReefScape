// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Lift extends SubsystemBase {

  private final TalonFX leftMotor, rightMotor;
  private final ElevatorFeedforward liftFeedforward;
  private final TrapezoidProfile elevatorVoltageProfile;
  private final Timer timer;

  // sysid
  //measures for the left motor
  private final MutVoltage appliedVoltageLeft = Volts.mutable(0);
  private final MutAngle liftDisplacementLeft = Radians.mutable(0);
  private final MutAngularVelocity liftVelocityLeft = RadiansPerSecond.mutable(0);
  private final MutAngularAcceleration liftAccelerationLeft = RadiansPerSecondPerSecond.mutable(0);
  //measures for the right motor
  private final MutVoltage appliedVoltageRight = Volts.mutable(0);
  private final MutAngle liftDisplacementRight = Radians.mutable(0);
  private final MutAngularVelocity liftVelocityRight = RadiansPerSecond.mutable(0);
  private final MutAngularAcceleration liftAccelerationRight = RadiansPerSecondPerSecond.mutable(0);
  private final SysIdRoutine routine;

  /** Creates a new Lift. */
  public Lift() {
    //configure motors 
    leftMotor = new TalonFX(Constants.LiftConstants.LEFT_LIFT_MOTOR_ID);
    rightMotor = new TalonFX(Constants.LiftConstants.RIGHT_LIFT_MOTOR_ID);
    //configure Feedforward and motion profies
    liftFeedforward = new ElevatorFeedforward(
      Constants.LiftConstants.LIFT_MOTOR_CONFIG.kS,
      Constants.LiftConstants.LIFT_MOTOR_CONFIG.kG,
      Constants.LiftConstants.LIFT_MOTOR_CONFIG.kV,
      Constants.LiftConstants.LIFT_MOTOR_CONFIG.kA
      );
    elevatorVoltageProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        Constants.LiftConstants.MAX_VELOCITY,
        Constants.LiftConstants.MAX_ACCELERATION
        )
    );
    timer = new Timer();
    //Configure Motors (should not affect sysid)
    rightMotor.getConfigurator().apply(Constants.LiftConstants.LIFT_MOTOR_CONFIG);
    leftMotor.getConfigurator().apply(Constants.LiftConstants.LIFT_MOTOR_CONFIG);
    
    //configure Sysid

    // NOTE::
    // anything with  variable -> {*some other stuff*}
    // is called a consumer. These are functions that do something but return nothing.
    // In sysid, all the functions do is either set motor voltages or write data to logs
    // so consumers make things more efficient
    
    routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      voltage ->{
        //set the voltage of the two motors
        leftMotor.setVoltage(voltage.magnitude());
        rightMotor.setVoltage(voltage.magnitude());
      }, log ->{
        //see the elevator identification section of the wpi docs for an explanation of why these parameters need to be logged
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
  public void setMotorStates(
    TrapezoidProfile.State currentRight,TrapezoidProfile.State currentLeft,
    TrapezoidProfile.State nextRight, TrapezoidProfile.State nextLeft) {
    leftMotor.setControl(
      new PositionDutyCycle(nextLeft.position)
      .withPosition(currentLeft.position)
      .withFeedForward(
        liftFeedforward.calculateWithVelocities(
          currentLeft.velocity
          ,nextLeft.velocity)
          / RobotController.getBatteryVoltage() //devide by battery voltage to normalize feedforward to [-1, 1]
          )
        );
    rightMotor.setControl(
      new PositionDutyCycle(nextRight.position)
      .withPosition(currentRight.position)
      .withFeedForward(
        liftFeedforward.calculateWithVelocities(
          currentRight.velocity
          ,nextRight.velocity)
          / RobotController.getBatteryVoltage()
          )
        );
  }
  /**
   * Dynamic Sysid test for the lift - will run the motors at at 7v (~60%) power until the test is stopped 
   * @param direction
   * @return Command
   */
  public Command liftSysidDynamic(SysIdRoutine.Direction direction){
    return routine.dynamic(direction);
  }
  /**
   * Quasistatic Sysid test for the lift - will run the motors at an increasing voltage up to 7v at a 0.5V step
   * @param direction
   * @return
   */
  public Command liftSysidQuasistatic(SysIdRoutine.Direction direction){
    return routine.quasistatic(direction);
  }
  private double initialLeftDistance, initialRightDistance;
  /**
   * executes a trapazoid profile to run the elevator up to a desired height 
   * @param relativeDistanceTicks
   *  - do (distance * gear ratio * final gear circumference / 2048) to convert inches of height to ticks
   * @return Command
   */
  //                                                                                   ___keep units the same__
  //                                                                                  |                        |
  //                                                                                  v                        v 
  public Command liftToDistanceProfiled(double relativeDistanceTicks /*(distance * gear ratio * final gear circumference * 2048)*/){ //TODO:: get the gear ratio and convert this parameter to desired height to raise to
    return startRun(()-> {
      timer.restart(); //restart timer so motion profile starts at the beginning
      initialLeftDistance = leftMotor.getPosition().getValueAsDouble() * 2048 /*convert rotations to ticks*/;
      initialRightDistance = rightMotor.getPosition().getValueAsDouble() * 2048;
    }, 
    ()-> {
      double currentTime = timer.get();
      TrapezoidProfile.State currentLeftSetpoint = 
      elevatorVoltageProfile.calculate(currentTime, new TrapezoidProfile.State(initialLeftDistance, 0), new TrapezoidProfile.State(relativeDistanceTicks, 0)); // remove the initalDistance from the desired final state of the profile to make the ticks absolute
      TrapezoidProfile.State currentRightSetpoint = 
      elevatorVoltageProfile.calculate(currentTime, new TrapezoidProfile.State(initialRightDistance, 0), new TrapezoidProfile.State(relativeDistanceTicks, 0));
      TrapezoidProfile.State desiredStateLeft = 
      elevatorVoltageProfile.calculate(currentTime + Constants.LiftConstants.Dt, new TrapezoidProfile.State(initialLeftDistance, 0), new TrapezoidProfile.State(relativeDistanceTicks, 0));
      TrapezoidProfile.State desiredStateRight = 
      elevatorVoltageProfile.calculate(currentTime + Constants.LiftConstants.Dt, new TrapezoidProfile.State(initialRightDistance, 0), new TrapezoidProfile.State(relativeDistanceTicks, 0));
      setMotorStates(currentRightSetpoint, currentLeftSetpoint, desiredStateRight, desiredStateLeft);
    }).until(()-> elevatorVoltageProfile.isFinished(0));
  }
  /**
   * this will kill every command running on the robot when it is executed. use with caution bc I dont know what will happen
   * @return
   */

public Command scrimageSetup() {
  return runOnce(()-> {
    leftMotor.set(.5);
      rightMotor.set(.5);
  });
}

public Command liftStop() {
  return runOnce(()-> {
    leftMotor.set(.0);
      rightMotor.set(0);
  });
}



  public Command hardStopAll_DANGER() {
    return runOnce(()-> {
      CommandScheduler.getInstance().cancelAll(); // this will
      leftMotor.setVoltage(0);
      rightMotor.setVoltage(0);
    });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
