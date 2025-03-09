package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
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

  public Arm() {
    armMotor = new TalonFX(Constants.ArmConstants.ARM_MOTOR_ID);

    armMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitThreshold(Rotations.of(21))
        .withReverseSoftLimitThreshold(Rotations.of(-.2))
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true));
    armMotor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration config = new TalonFXConfiguration()
      .withSlot0(Constants.ArmConstants.ARM_MOTOR_CONFIG)
      .withMotionMagic(Constants.ArmConstants.ARM_MOTION_CONFIGS);
    armMotor.getConfigurator().apply(config);
    armMotor.setNeutralMode(NeutralModeValue.Brake);

    armFeedForward = new ArmFeedforward(Constants.ArmConstants.ARM_MOTOR_CONFIG.kS, Constants.ArmConstants.ARM_MOTOR_CONFIG.kG, Constants.ArmConstants.ARM_MOTOR_CONFIG.kV);

    // sysid
    armMotorVoltage = Volts.mutable(0);
    armMotorAngle = Radians.mutable(0);
    armMotorVelo = RadiansPerSecond.mutable(0);
    armMotorAccel = RadiansPerSecondPerSecond.mutable(0);
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
    // feedforward needs to convert from arm motor rotations to arm gearbox output position in radians rotated 90 degrees
    // math : (motor rotations / gear ratio) * (2 * PI) + (PI/2)
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
    return (armPosition()- 12.6)/46.69; // ?? (jack)
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
