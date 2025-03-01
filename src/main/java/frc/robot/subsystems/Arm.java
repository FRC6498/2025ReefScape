// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
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
  private final ArmFeedforward armFeedForward;


  //TODO:: Configure an offset for the arm motor so 0 in at the intake position

  public Arm() {
    armMotor = new TalonFX(Constants.ArmConstants.ARM_MOTOR_ID);
    // sysid
    armMotorVoltage = Volts.mutable(0);
    armMotorAngle = Radians.mutable(0);
    armMotorVelo = RadiansPerSecond.mutable(0);
    armMotorAccel = RadiansPerSecondPerSecond.mutable(0);

    armMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitThreshold(Rotations.of(21))
        .withReverseSoftLimitThreshold(Rotations.of(-.2)).withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true));
    armMotor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0 = Constants.ArmConstants.ARM_MOTOR_CONFIG;
    // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 20;
    // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    config.MotionMagic = Constants.ArmConstants.ARM_MOTION_CONFIGS;
    armMotor.getConfigurator().apply(config);
    armMotor.setNeutralMode(NeutralModeValue.Brake);

    armFeedForward = new ArmFeedforward(Constants.ArmConstants.ARM_MOTOR_CONFIG.kS, Constants.ArmConstants.ARM_MOTOR_CONFIG.kG, Constants.ArmConstants.ARM_MOTOR_CONFIG.kV);
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
            this));
  }

  public Command armSysidQuasistatic(SysIdRoutine.Direction direction) {
    return armRoutine.quasistatic(direction);
  }

  public Command armSysidDynamic(SysIdRoutine.Direction direction) {
    return armRoutine.dynamic(direction);
  }

  public Command runToRotationsMagic(double setpointRotations) {
    MotionMagicVoltage request = new MotionMagicVoltage(armMotor.getPosition().getValueAsDouble());
    double ff = armFeedForward.calculate((setpointRotations/46.69 - 12.6)*2*Math.PI, 0);
    SmartDashboard.putNumber("ff", ff);
    return run(() -> 
    armMotor.setControl(
      request.withPosition(setpointRotations)
        .withFeedForward(ff)
      )
    );
  }

  public double getMotionMagic() {
    return armMotor.getMotionMagicIsRunning().getValueAsDouble();
  }

  public Command stopArm() {

    return runOnce(() -> armMotor.set(0));
  }

  public BooleanSupplier canRaise() {
    return () -> armMotor.getPosition().getValueAsDouble() > 4;
  }

  public double armPosition() {
    return armMotor.getPosition().getValueAsDouble();
  }

  public double getRealArmRotation(){
    return (armPosition()- 12.6)/46.69;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("mmmmmmm", getMotionMagic());
    SmartDashboard.putNumber("Arm Postiion", armPosition());
    SmartDashboard.putNumber("realarmpos", getRealArmRotation()*360);

    SmartDashboard.putNumber("V", armMotor.getMotorVoltage().getValueAsDouble());

    double ff = armFeedForward.calculate((armPosition()/46.69 - 12.6)*2*Math.PI, 0);
    SmartDashboard.putNumber("ff", ff);

  }
}
