// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX intakeMotor;
  public Intake() {
    intakeMotor = new TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    intakeMotor.get();
  }
  /**
   * Runs the intake at speed IntakeConstants.INTAKE_DEFAULT_SPEED
   * @return
   * Command
   */
  public Command runIntake(){
    return this.runOnce(()-> {intakeMotor.set(Constants.IntakeConstants.INTAKE_DEFAULT_SPEED);});
  }
  /**
   * Runs the intake at a custom speed
   * @param speed
   * (double) speed to run the intake at (from -1 to 1)
   * @return
   * Command
   */
  public Command runIntake(double speed){
    return this.runOnce(()-> {intakeMotor.set(speed);});
  }
  /**
   * stops the intake
   * @return
   * Command
   */
  public Command stopIntake(){
    return this.runOnce(()-> {intakeMotor.set(0);});
  }
  /**
   * Gets the speed that the intake is currently running at
   * 
   * @return
   * double
   */
  @Logged
  public double getSpeed() {
    return intakeMotor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
