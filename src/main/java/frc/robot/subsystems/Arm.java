// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.MutAngularAcceleration;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutAcceleration;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
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
  /** Creates a Arm */
  public Arm() {
    armFeedforward = new ArmFeedforward(Constants.ArmConstants.ARM_MOTOR_CONFIG.kS, Constants.ArmConstants.ARM_MOTOR_CONFIG.kG, Constants.ArmConstants.ARM_MOTOR_CONFIG.kV);
    armMotor = new TalonFX(Constants.ArmConstants.ARM_MOTOR_ID);
    //sysid
    armMotorVoltage = Volts.mutable(0);
    armMotorAngle = Radians.mutable(0);
    armMotorVelo = RadiansPerSecond.mutable(0);
    armMotorAccel = RadiansPerSecondPerSecond.mutable(0);

  }
  

  public Command runToPosition(double positionTicks){
    return run(()-> {
      armMotor.setControl(
        new PositionDutyCycle(positionTicks) // where you want to go
        .withPosition(armMotor.getPosition().getValueAsDouble()) //where you are 
        .withFeedForward(
          armFeedforward.calculate(
            armMotor.getPosition().getValueAsDouble(),
            armMotor.getVelocity().getValueAsDouble()
            )
        )
      );
    });
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
