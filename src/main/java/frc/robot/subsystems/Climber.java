// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX climbMotor = new TalonFX(26);
  public Climber() {

    // todo get PID
    // Slot0Configs slot0Config = new Slot0Configs();
    // climbMotor.getConfigurator()

    climbMotor.setNeutralMode(NeutralModeValue.Brake);
    climbMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitThreshold(Rotations.of(380))
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(false));

    
  }

  public Command runForward(){
    return run(()->climbMotor.setVoltage(3));
  }

  public Command runReverse(){
    return run(()->climbMotor.setVoltage(-3));
  }

  public Command stop(){
    return runOnce(()->climbMotor.stopMotor());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("climber position", climbMotor.getPosition().getValueAsDouble());
  }
}
