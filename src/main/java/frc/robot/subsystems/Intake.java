// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX intakeMotor;
  private final CANrange armSensor; 

  public Intake() {
    armSensor = new CANrange(Constants.IntakeConstants.CANRANGE_SENSOR_ID);
    intakeMotor = new TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    intakeMotor.get();
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }
  /**
   * Runs the intake at speed IntakeConstants.INTAKE_DEFAULT_SPEED
   * @return
   * Command
   */

   public Command ejectIntake(){
    return this.run(()-> {intakeMotor.set(Constants.IntakeConstants.INTAKE_DEFAULT_SPEED);});
   }


   public Command ejectAlgae(){
    return this.run(()-> {intakeMotor.set(Constants.IntakeConstants.INTAKE_ALGAE_SPEED);});
   }

  public Command runIntake(){
    return this.run(()-> {intakeMotor.set(Constants.IntakeConstants.INTAKE_DEFAULT_SPEED);})
               .until(intakeStop())
               .andThen(stopIntake());
  }

  public Command intakeAlgaeCommand() {
    return this.run(()-> {intakeMotor.set(.2);});
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
   * Gets the distance coral is from the range 
   * 
   * @return
   * double
   */
@Logged
public double getDistance() {
  return armSensor.getDistance().getValueAsDouble();
}

 public BooleanSupplier intakeStop() {
    
    return () -> (getDistance() < .1 && armSensor.getIsDetected().getValue());

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
    SmartDashboard.putBoolean("detected", intakeStop().getAsBoolean());
  }
}
