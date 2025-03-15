// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LinearActuator;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX climbMotor = new TalonFX(Constants.ClimberConstants.CLIMB_MOTOR_ID);
  private LinearActuator latchActuator = new LinearActuator(Constants.ClimberConstants.ACTUATOR_PWM_PORT, 140);
  private DigitalOutput solenoid = new DigitalOutput(0);

  public Climber() {

    // todo get PID
    // Slot0Configs slot0Config = new Slot0Configs();
    // climbMotor.getConfigurator()

    climbMotor.setNeutralMode(NeutralModeValue.Brake);
    climbMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitThreshold(Rotations.of(380))
        .withForwardSoftLimitEnable(false)
        .withReverseSoftLimitEnable(false));

    
  }

  public Command runForward(){
    return run(()->climbMotor.set(Constants.ClimberConstants.MOTOR_SPEED));
  }

  public Command runReverse(){
    return run(()->climbMotor.set(-Constants.ClimberConstants.MOTOR_SPEED));
  }

  public Command stop(){
    return runOnce(()->climbMotor.stopMotor()); 
  }

  public Command unlatch(){
    return runOnce(()->latchActuator.setLength(0));
  }

  public Command unlatchSolenoid(){
    return runOnce(()->solenoid.set(true));
  }

  public Command reverseLatch(){
    return runOnce(()->latchActuator.setLength(Inches.of(1.9)));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("climber position", climbMotor.getPosition().getValueAsDouble());
  }
}