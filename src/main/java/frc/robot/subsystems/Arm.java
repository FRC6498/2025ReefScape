// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
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
  private final MutVoltage armMotorVoltage;
  private final MutAngle armMotorAngle;
  private final MutAngularVelocity armMotorVelo; 
  private final MutAngularAcceleration armMotorAccel; 
  private final SysIdRoutine armRoutine;
  /** Creates a Arm */

  //TODO:: Configure an offset for the arm motor so 0 in at the intake position

  public Arm() {
    
    armMotor = new TalonFX(Constants.ArmConstants.ARM_MOTOR_ID);
    //sysid
    armMotorVoltage = Volts.mutable(0);
    armMotorAngle = Radians.mutable(0);
    armMotorVelo = RadiansPerSecond.mutable(0);
    armMotorAccel = RadiansPerSecondPerSecond.mutable(0);
    
    armMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitThreshold(Rotations.of(9))
    .withReverseSoftLimitThreshold(Rotations.of(-.2)).withForwardSoftLimitEnable(true)
    .withReverseSoftLimitEnable(true));
    armMotor.setNeutralMode(NeutralModeValue.Brake);
    armMotor.getConfigurator().apply(Constants.ArmConstants.ARM_MOTOR_CONFIG);
    
    
    
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

  public Command runToRotationsMagic(double setpointRotations) {
    MotionMagicVoltage request = new MotionMagicVoltage(setpointRotations);
    return run(()-> armMotor.setControl(request));
  }
  
  public Command stopArm() {
    return runOnce(()-> armMotor.setVoltage(0));
  }

  public double armPosition() {
    return armMotor.getPosition().getValueAsDouble();
  }

  public Command scrimageArmForward() {
    return runOnce(() -> armMotor.set(.1));
  }

  public Command scrimageArmBackward() {
    return runOnce(() -> armMotor.set(-0.1));
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
