// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Intake intakeSub;
    private final Arm armSub;
    private final Lift liftSub;

    public RobotContainer() {
        intakeSub = new Intake();
        liftSub = new Lift();
        armSub = new Arm();

        // armSub.setDefaultCommand();
        configureBindings();
    }

    public Command intakeCoral() {
        return (intakeSub.runIntake().until(intakeSub.intakeStop()).andThen(intakeSub.stopIntake()));
    }

    public Command intakeAlgae() {
        return (intakeSub.intakeAlgaeCommand());
    }

    public Command ejectIntake() {
        return (intakeSub.ejectIntake());
    }

    public Command stopIntakeCoral() {
        return (intakeSub.stopIntake());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive
                                                                                                          // forward
                                                                                                          // with
                                                                                                          // negative Y
                                                                                                          // (forward)
                        .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                          // negative X (left)
                ));

        driveController.x().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(.5)
                .withVelocityY(0)
                .withRotationalRate(0)));

        driveController.y().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(.5)));

        driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driveController.b().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        operatorController.x().whileTrue(intakeSub.runIntake()).whileFalse(intakeSub.stopIntake());
        operatorController.y().whileTrue(intakeSub.ejectIntake()).whileFalse(intakeSub.stopIntake());
        operatorController.leftTrigger().whileTrue(intakeSub.intakeAlgaeCommand()).whileFalse(intakeSub.stopIntake());
        operatorController.a().onTrue(armSub.runToRotationsMagic(17));//.whileFalse((stopArm()));
        operatorController.b().onTrue(armSub.runToRotationsMagic(0));//.whileFalse((stopArm()));
        operatorController.start().onTrue(liftSub.zeroLift());//.whileFalse(liftStop());
        operatorController.rightBumper().whileTrue(armSub.runToRotationsMagic(5).unless(armSub.canRaise())
            .until(armSub.canRaise()).andThen(liftSub.scrimageSetup(.1))).whileFalse(liftStop());

        operatorController.povDown().onTrue(lift(0));
        operatorController.povLeft().onTrue(lift(7));
        operatorController.povRight().onTrue(lift(17));
        operatorController.povUp().onTrue(lift(30));

    }
        
    public Command lift(double rotations){

        return armSub.runToRotationsMagic(5).unless(armSub.canRaise())
        .until(armSub.canRaise()).andThen(liftSub.runToRotations(rotations));
    }

    public Command liftHold() {
        return (liftSub.liftHold());
    }

    public Command liftStop() {
        return (liftSub.liftStop());
    }

    public Command testArm() {
        return armSub.runToRotationsMagic(4);
    }

    public Command reverseArmSysidDynamic() {
        return (armSub.armSysidDynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command reversearmSysidQuasistatic() {
        return (armSub.armSysidQuasistatic(SysIdRoutine.Direction.kReverse));
    }
    public Command forwardArmSysidDynamic() {
        return (armSub.armSysidDynamic(SysIdRoutine.Direction.kForward));
    }

    public Command forwardarmSysidQuasistatic() {
        return (armSub.armSysidQuasistatic(SysIdRoutine.Direction.kForward));
    }

    public Command stopArm() {
        return (armSub.stopArm());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
