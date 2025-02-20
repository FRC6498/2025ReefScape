// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private final TalonFX armMotor;
  private final ArmFeedforward armFeedforward;
  private final MutVoltage armMotorVoltage;
  private final MutAngle armMotorAngle;
  private final MutAngularVelocity armMotorVelo; 
  private final MutAngularAcceleration armMotorAccel; 
  private final SysIdRoutine armRoutine;
  private final Timer armTimer;
  private final TrapezoidProfile armVoltageProfile;
  /** Creates a Arm */

  //TODO:: Configure an offset for the arm motor so 0 in at the intake position

  public Arm() {
    armFeedforward = new ArmFeedforward(
      Constants.ArmConstants.ARM_MOTOR_CONFIG.kS,
      Constants.ArmConstants.ARM_MOTOR_CONFIG.kG, 
      Constants.ArmConstants.ARM_MOTOR_CONFIG.kV
    );
    armMotor = new TalonFX(Constants.ArmConstants.ARM_MOTOR_ID);
    //sysid
    armMotorVoltage = Volts.mutable(0);
    armMotorAngle = Radians.mutable(0);
    armMotorVelo = RadiansPerSecond.mutable(0);
    armMotorAccel = RadiansPerSecondPerSecond.mutable(0);
    
    armMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitThreshold(Degrees.of(19))
    .withReverseSoftLimitThreshold(Degrees.of(-90)).withForwardSoftLimitEnable(true)
    .withReverseSoftLimitEnable(true));
    armMotor.setNeutralMode(NeutralModeValue.Brake);
    armMotor.getConfigurator().apply(Constants.ArmConstants.ARM_MOTOR_CONFIG);
    
    armTimer = new Timer();    
    armVoltageProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        Constants.ArmConstants.ARM_MAX_VELOCITY, 
        Constants.ArmConstants.ARM_MAX_ACCELERATION
      )
    );
    
    armRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(), 
      new SysIdRoutine.Mechanism(
        voltage -> {
          armMotor.setVoltage(voltage.magnitude());
        }, 
        log -> {
          log.motor("armMotor")
          .voltage(armMotorVoltage.mut_replace(armMotor.getMotorVoltage().getValue()))
          .angularVelocity(armMotorVelo.mut_replace(armMotor.getVelocity().getValue()))
          .angularAcceleration(armMotorAccel.mut_replace(armMotor.getAcceleration().getValue()))
          .angularPosition(armMotorAngle.mut_replace(armMotor.getPosition().getValue()));
        },
      this
      )
    );    
  }
  public Command armSysidQuasistatic(SysIdRoutine.Direction direction) {
    return armRoutine.quasistatic(direction);
  }  
  public Command armSysidDynamic(SysIdRoutine.Direction direction) {
    return armRoutine.dynamic(direction);
  }

//tbh im not sure that all of the units are correct across all of these methods so if this does not work then check that
  public void setMotorStates(TrapezoidProfile.State current,TrapezoidProfile.State next) {
    armMotor.setControl(
      new PositionDutyCycle(next.position)
      .withPosition(current.position)
      .withFeedForward(
        armFeedforward.calculateWithVelocities((-Math.PI/2 + Rotations.of(armMotor.getPosition().getValue().magnitude()).in(Radians)), current.velocity,next.velocity)
          / RobotController.getBatteryVoltage() //devide by battery voltage to normalize feedforward to [-1, 1]
          )
        );
  }
  private double initalDistance = 0;
  public Command runToAngleProfiled(Angle angle){ 
    return startRun(()-> {
      armTimer.restart(); //restart timer so motion profile starts at the beginning
      initalDistance = armMotor.getPosition().getValueAsDouble() * 2048;
    }, 
    ()-> {
      double currentTime = armTimer.get();
      TrapezoidProfile.State currentSetpoint = 
      armVoltageProfile.calculate(currentTime, new TrapezoidProfile.State(initalDistance, 0), new TrapezoidProfile.State(0, 0)); // remove the initalDistance from the desired final state of the profile to make the ticks absolute
      TrapezoidProfile.State desiredState = 
      armVoltageProfile.calculate(currentTime + Constants.ArmConstants.Dt, new TrapezoidProfile.State(initalDistance, 0), new TrapezoidProfile.State(angle.magnitude(), 0));
      setMotorStates(currentSetpoint, desiredState);
    }).until(()-> armVoltageProfile.isFinished(0));
  }

  public Command runToAngle(Angle angle){ // 0 is assumed to be the intake position
    angle.times(46.69);
    return run(()-> {
      armMotor.setControl(
        new PositionVoltage(angle.in(Rotations)) // where you want to go
        .withPosition(armMotor.getPosition().getValue().in(Rotations)) //where you are 
        .withFeedForward(
          armFeedforward.calculate(
           Math.PI/2* - angle.in(Radians), //angle 0 has to be horizontal
            0
            )
        )
      );
    });
  }


  public Command stopArm() {
    return runOnce(()-> armMotor.setVoltage(0));
  }

  public double armPosition() {
    return armMotor.getPosition().getValueAsDouble();
  }
  // runToPosition(10);
  // runToPosition(20);
  // runToPosition(30);
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Postiion", armPosition());
  }
}
