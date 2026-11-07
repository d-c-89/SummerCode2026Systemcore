// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// random comment! 

package frc.robot;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.telemetry.Telemetry;
import frc.robot.SwerveTelemetry;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.command2.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.operatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.pathplanner.lib.auto.NamedCommands;
import org.wpilib.driverstation.Gamepad;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeMoverSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.commands.OutTakeKickerHopper;
import frc.robot.commands.RunIntakeHopper;
import frc.robot.commands.RunIntakeHopperOut;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootOnTheMove;
import frc.robot.commands.RunIntake;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RetractIntake;
import org.wpilib.tunable.Selectable;


//import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    
    private final Selectable<Command> autoChooser;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveTelemetry logger = new SwerveTelemetry(MaxSpeed);

    private final CommandGamepad driverController0 = new CommandGamepad(0);
    private final CommandGamepad driverController1 = new CommandGamepad(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //subsystem and command declaration 
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    private final TurretSubsystem m_TurretSubsystem = new TurretSubsystem();
    private final HopperSubsystem m_HopperSubsystem = new HopperSubsystem();
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final KickerSubsystem m_KickerSubsystem = new KickerSubsystem();
    private final IntakeMoverSubsystem m_IntakeMoverSubsystem = new IntakeMoverSubsystem();

    

    public RobotContainer() {

        //Named commands for auto
        //these commands MUST have end condition in "isFInished" part of command for it to be able to move on to the next step in pathplanner



        //drivetrain.initTelemetry(); 
        //REMOVED. ABOVE LINE WAS PRESENT IN 2026 CODE
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        Telemetry.log("Auto", autoChooser);
    }

    private void configureBindings() {
        //named commands for pathplanner:
        NamedCommands.registerCommand("Shoot3Sec", new Shoot(m_ShooterSubsystem, m_TurretSubsystem, drivetrain, m_KickerSubsystem, m_HopperSubsystem, 3));
        NamedCommands.registerCommand("Shoot5Sec", new Shoot(m_ShooterSubsystem, m_TurretSubsystem, drivetrain, m_KickerSubsystem, m_HopperSubsystem, 5));
        NamedCommands.registerCommand("ShootOnTheMove3Sec", new ShootOnTheMove(m_ShooterSubsystem, m_TurretSubsystem, drivetrain, m_KickerSubsystem, m_HopperSubsystem, 3));
        NamedCommands.registerCommand("ShootOnTheMove5Sec", new ShootOnTheMove(m_ShooterSubsystem, m_TurretSubsystem, drivetrain, m_KickerSubsystem, m_HopperSubsystem, 5));
         NamedCommands.registerCommand("ShootOnTheMove10Sec", new ShootOnTheMove(m_ShooterSubsystem, m_TurretSubsystem, drivetrain, m_KickerSubsystem, m_HopperSubsystem, 10));
        NamedCommands.registerCommand("RunHopper3Sec", new RunIntakeHopper(m_HopperSubsystem, m_IntakeSubsystem));
        NamedCommands.registerCommand("RunIntake4Sec", m_IntakeSubsystem.runIntake().withTimeout(4));
        NamedCommands.registerCommand("deployIntake", new DeployIntake(m_IntakeMoverSubsystem));
        NamedCommands.registerCommand("RetractIntake", new RetractIntake(m_IntakeMoverSubsystem));

        int aButton = 1;
        int bButton = 2;
        int xButton = 3;
        int yButton = 4;
        int povleftButton = 14;
        int povrightButton = 13;
        int povupButton = 11;

        //DRIVETRAIN KEYBINDS
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController0.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController0.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController0.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController0.button(aButton).whileTrue(drivetrain.applyRequest(() -> brake));
        driverController0.button(bButton).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController0.getLeftY(), -driverController0.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        driverController0.back().and(driverController0.button(yButton)).whileTrue(drivetrain.sysIdDynamic(Direction.FORWARD));
        driverController0.back().and(driverController0.button(xButton)).whileTrue(drivetrain.sysIdDynamic(Direction.REVERSE));
        driverController0.start().and(driverController0.button(yButton)).whileTrue(drivetrain.sysIdQuasistatic(Direction.FORWARD));
        driverController0.start().and(driverController0.button(xButton)).whileTrue(drivetrain.sysIdQuasistatic(Direction.REVERSE));

        // Reset the field-centric heading on left bumper press.
        //driverController0.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        //COMMAND KEYBINDS
        //CONTROLLER 0
        driverController0.leftTrigger().whileTrue(m_IntakeSubsystem.runIntake());
        driverController0.button(xButton).whileTrue(new RunIntakeHopperOut(m_HopperSubsystem, m_IntakeSubsystem));
        driverController0.button(yButton).whileTrue(new OutTakeKickerHopper(m_HopperSubsystem, m_KickerSubsystem));

        driverController0.rightBumper().whileTrue(m_IntakeMoverSubsystem.intakeMoverSpeed(0.3));
        driverController0.leftBumper().whileTrue(m_IntakeMoverSubsystem.intakeMoverSpeed(-0.3));
        driverController0.rightTrigger().whileTrue(new ShootOnTheMove(m_ShooterSubsystem, m_TurretSubsystem, drivetrain, m_KickerSubsystem, m_HopperSubsystem, 0));
        driverController0.button(povleftButton).onTrue(m_TurretSubsystem.offsetLeft());
        driverController0.button(povrightButton).onTrue(m_TurretSubsystem.offsetRight());
        driverController0.button(povupButton).onTrue(m_TurretSubsystem.offsetZero());

        //CONTROLLER 1
        
        //driverController1.x().whileTrue();
        driverController1.leftTrigger().whileTrue(m_IntakeSubsystem.runIntake());
        driverController1.button(xButton).whileTrue(new RunIntakeHopperOut(m_HopperSubsystem, m_IntakeSubsystem));
        driverController1.button(yButton).whileTrue(new OutTakeKickerHopper(m_HopperSubsystem, m_KickerSubsystem));
        driverController1.button(aButton).whileTrue(new Shoot(m_ShooterSubsystem, m_TurretSubsystem, drivetrain, m_KickerSubsystem, m_HopperSubsystem, 0));

        driverController1.rightBumper().whileTrue(m_IntakeMoverSubsystem.intakeMoverSpeed(0.3));
        driverController1.leftBumper().whileTrue(m_IntakeMoverSubsystem.intakeMoverSpeed(-0.3));
        
        driverController1.rightTrigger().whileTrue(new ShootOnTheMove(m_ShooterSubsystem, m_TurretSubsystem, drivetrain, m_KickerSubsystem, m_HopperSubsystem, 0));
        driverController1.button(povleftButton).onTrue(m_TurretSubsystem.offsetLeft());
        driverController1.button(povrightButton).onTrue(m_TurretSubsystem.offsetRight());
        driverController1.button(povupButton).onTrue(m_TurretSubsystem.offsetZero());


        
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        drivetrain.runOnce(drivetrain::seedFieldCentric);
        //return new PathPlannerAuto("Example Auto");
        return autoChooser.getSelected();
    }
}
