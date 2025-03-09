package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second

    // Drive Command configs    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain;
    private final Intake intakeSub;
    private final Arm armSub;
    private final Lift liftSub;
    private final Telemetry logger; 
    
    private final SendableChooser<Command> chooser;

    public RobotContainer() {
        drivetrain = TunerConstants.createDrivetrain();
        intakeSub = new Intake();
        liftSub = new Lift();
        armSub = new Arm();
        logger = new Telemetry(MaxSpeed);
        // Pathplanner Commands
        NamedCommands.registerCommand("Run Intake", intakeSub.runIntake());
        NamedCommands.registerCommand("Stop Intake", intakeSub.stopIntake());
        NamedCommands.registerCommand("ejectIntake", intakeSub.stopIntake());

        
        chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("autoChooser", chooser);

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-driveController.getLeftY() * MaxSpeed) 
                .withVelocityY(-driveController.getLeftX() * MaxSpeed) 
                .withRotationalRate(driveController.getRightX() * MaxAngularRate) 
            )
        );
        driveController.x().whileTrue(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(.5)
                .withVelocityY(0)
                .withRotationalRate(0)
            )
        );
        driveController.y().whileTrue(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(.5)
            )
        );
        driveController.a().whileTrue(
            drivetrain.applyRequest(() -> brake)
            );
        driveController.b().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(
                    new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX())
                )
        ));

        // reset the field-centric heading on left bumper press
        driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        // Coral Intake
        operatorController.leftBumper().whileTrue(intakeSub.runIntake()).whileFalse(intakeSub.stopIntake());

        // Coral eject
        operatorController.rightBumper().whileTrue(intakeSub.ejectIntake()).whileFalse(intakeSub.stopIntake());

        // Algae Intake
        operatorController.leftTrigger().whileTrue(intakeSub.intakeAlgaeCommand()).whileFalse(intakeSub.stopIntake());

        // Run arm to Algea Position
        operatorController.a().onTrue(armSub.runToRotationsMagic(17));

        // Run arm to zero
        // stops arms so motion magic always returns to zero
        operatorController.b().onTrue(armSub.stopArm().andThen(armSub.runToRotationsMagic(0)));

        // Run arm to Algea eject Position
        operatorController.start().onTrue(armSub.runToRotationsMagic(21));
        
        // 
        operatorController.x().whileTrue(armSub.runToRotationsMagic(5).unless(armSub.canRaise())
            .until(armSub.canRaise()).andThen(liftSub.scrimageSetup(.1))).whileFalse(liftSub.liftStop());
        
        // Algea eject
        operatorController.rightTrigger().onTrue(intakeSub.ejectAlgae()).onFalse(intakeSub.stopIntake());
        
        // Run lift to 0
        operatorController.povDown().onTrue(liftSub.runToRotations(0));
        
        // Run lift to level 1
        operatorController.povLeft().onTrue(liftSub.runToRotations(7));
        
        // Run lift to level 2
        operatorController.povRight().onTrue(liftSub.runToRotations(16));
        
        // Run lift to level 3
        operatorController.povUp().onTrue(liftSub.runToRotations(30));
        
        // Eject algea to barge
        // Run lift to max and Rotate arm to Algea eject position
        operatorController.y().onTrue(liftSub.runToRotations(33.8).andThen(armSub.runToRotationsMagic(14)));

    }
        
    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

}
