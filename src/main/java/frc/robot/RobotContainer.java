package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
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
    private final Climber climberSub;
    private final Telemetry logger; 
    
    private final SendableChooser<Command> chooser;

    public RobotContainer() {
        drivetrain = TunerConstants.createDrivetrain();
        intakeSub = new Intake();
        liftSub = new Lift();
        armSub = new Arm();
        climberSub = new Climber();
        logger = new Telemetry(MaxSpeed);

        // auto zero lift
        CommandScheduler.getInstance().schedule(
            liftSub.liftStop().
            withTimeout(.1).
            andThen(liftSub.zeroLift()).
            andThen(armSub.zeroArm())
        );

        // Pathplanner Commands
        NamedCommands.registerCommand("Run Intake", intakeSub.runIntake().until(intakeSub.intakeStop()));
        NamedCommands.registerCommand("Stop Intake", intakeSub.stopIntake());
        NamedCommands.registerCommand("Algae Intake", intakeSub.intakeAlgaeCommand().withTimeout(2).andThen(intakeSub.stopIntake()));
        NamedCommands.registerCommand("Eject Intake", intakeSub.ejectIntake().until(intakeSub.stopEject()));
        NamedCommands.registerCommand("Arm Out", armSub.runToRotationsMagic(5).withTimeout(.5));
        NamedCommands.registerCommand("Arm Algae", armSub.runToRotationsMagic(17).withTimeout(.3));
        NamedCommands.registerCommand("Arm Liftpos", armSub.runToRotationsMagic(2).withTimeout(.2));
        NamedCommands.registerCommand("Arm In", armSub.runToRotationsMagic(0).withTimeout(.1));
        NamedCommands.registerCommand("Lift Position One", safeLiftAuto(0).withTimeout(1.5).andThen(liftSub.liftStop()));
        NamedCommands.registerCommand("Lift Position Two", safeLiftAuto(7).withTimeout(.5));
        NamedCommands.registerCommand("Lift Position Three", safeLiftAuto(17).withTimeout(.5));
        NamedCommands.registerCommand("Lift Position Four", safeLiftAuto(30).withTimeout(1));

        
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

        // driveController.a().whileTrue(
        //     drivetrain.applyRequest(() -> brake)
        //     );
        // driveController.b().whileTrue(drivetrain.applyRequest(() -> point
        //         .withModuleDirection(
        //             new Rotation2d(-driveController.
        // getLeftY(), -driveController.getLeftX())
        //         )
        // ));z
    
        // reset the field-centric heading on left abumper press
        driveController.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // Slow mode
        // driveController.rightTrigger().onChange(drietrain.runOnce(() -> {
        //     MaxSpeed = MaxSpeed == 1 ? TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) : 1;
        // }));

        // climber
        driveController.rightBumper().whileTrue(climberSub.runForward()).onFalse(climberSub.stop()); // in
        driveController.leftBumper().whileTrue(climberSub.runReverse()).onFalse(climberSub.stop());  // out
        driveController.b().onTrue(climberSub.unlatch());
        driveController.povUp().onTrue(climberSub.reverseLatch()); // reset latch
        

        // Coral Intake
        operatorController.leftBumper().whileTrue(intakeSub.runIntake()).whileFalse(intakeSub.stopIntake());

        // Coral eject
        operatorController.rightBumper().whileTrue(intakeSub.ejectIntake()).whileFalse(intakeSub.stopIntake());

        // Algae Intake`
        operatorController.leftTrigger().whileTrue(intakeSub.intakeAlgaeCommand()).whileFalse(intakeSub.stopIntake());
        
        
        // Algea eject
        operatorController.rightTrigger().onTrue(intakeSub.ejectAlgae()).onFalse(intakeSub.stopIntake());

        // Run arm to Algea Position
        operatorController.a().onTrue(armSub.runToRotationsMagic(17));

        // Run arm to zero
        // stops arms so motion magic always returns to 
        

        operatorController.b().onTrue(armSub.goToZero());
        
        // homes arm and lift
        // moves lift down then arm in
        operatorController.x().onTrue(liftHome());
        
        // Eject algea to barge
        // Run lift to max and Rotate arm to Algea eject position
        operatorController.y().onTrue(safeLift(33).withTimeout(.5).andThen(armSub.runToRotationsMagic(15)));
    
        // Run arm to Algea eject Position
        operatorController.start().onTrue(armSub.runToRotationsMagic(21));
        // operatorController.start().whileTrue(liftSub.scrimageSetup(.1))
        //                           .whileFalse(liftSub.liftStop());
        // operatorController.start().onTrue(
        //     liftSub.liftStop().
        //     withTimeout(.1).
        //     andThen(liftSub.zeroLift()).
        //     andThen(armSub.zeroArm()));



        // Run lift to 0
        operatorController.povDown().onTrue(safeLift(0).until(liftSub.atBottom()).withTimeout(2).andThen(liftSub.liftStop()));
        
        // Run lift to level 1
        operatorController.povLeft().onTrue(safeLift(7));
        

        // Run lift to level 2
        operatorController.povRight().onTrue(safeLift(16));
        
        // Run lift to level 3
        operatorController.povUp().onTrue(safeLift(30));

        

    }

    // makes sure arm is out of the way before moving lift
    public Command safeLift(double rotations){
        return armSub.runToRotationsMagic(5)
                     .unless(armSub.canRaise())
                     .until(armSub.canRaise())
                     .withTimeout(2)
                     .andThen(liftSub.runToRotations(rotations));
                    //  .andThen(armSub.runToRotationsMagic(5));
    }

    public Command safeLiftAuto(double rotations){
        return armSub.runToRotationsMagic(2)
                     .unless(armSub.canRaise())
                     .until(armSub.canRaise())
                     .withTimeout(2)
                     .andThen(liftSub.runToRotations(rotations));
    }

    public Command liftHome(){
        return safeLift(0).
            withTimeout(2).
            until(liftSub.atBottom()).
            andThen(liftSub.liftStop().
            andThen(armSub.goToZero())
            );
    }
        
    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}

