// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KickerSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

import org.wpilib.telemetry.*;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.Alliance;
import org.wpilib.system.Timer;


import frc.robot.Constants.operatorConstants;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  ShooterSubsystem m_shooter;
  TurretSubsystem m_turret;
  KickerSubsystem m_kicker;
  HopperSubsystem m_hopper;
  private final Timer m_timer = new Timer();


  CommandSwerveDrivetrain m_drivetrain;
  private double ShootAngle;
  private double ShootRPS;
  private double time;


  //private final RunIntakeHopper m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot (ShooterSubsystem shooter, TurretSubsystem turret, CommandSwerveDrivetrain drivetrain, KickerSubsystem kicker, HopperSubsystem hopper, double time) {
    m_shooter = shooter;
    this.time = time;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    m_drivetrain = drivetrain;
    m_turret = turret;
    addRequirements(turret);

    m_kicker = kicker;
    addRequirements(kicker);

    m_hopper = hopper;
    addRequirements(hopper);
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();

    var alianceColor = MatchState.getAlliance();
    //calculate shooting speed and turret position
    Pose2d currentPose = m_drivetrain.getState().Pose;

    double x = currentPose.getX();
    double y = currentPose.getY();
    double hubX=0;
    double hubY=0;
    if (Alliance.RED == MatchState.getAlliance().get()){
        hubX = 11.901424;
        hubY = 4.021328;
    }
    else {
        hubX = 4.611624;
        hubY = 4.021328;
    }
    
    double g = -9.8; 
    double launchAngle = 1.27409; //use radians!!!
    double heightDiff = -1.2712; //shooter height-minus hub height (should be negative) 
    double shooterWheelRadius = 0.1016;

    
    

    double distanceToHub = Math.sqrt(Math.pow((hubX-x),2)+Math.pow((hubY-y),2));
    double launchVelocity = Math.sqrt((g*Math.pow(distanceToHub,2))/((2*Math.cos(launchAngle)*Math.cos(launchAngle))*(-heightDiff
    -distanceToHub*Math.tan(launchAngle))));

    double ShootRPS = operatorConstants.kShooterBoost*(launchVelocity*2)/(Math.PI*shooterWheelRadius);

    //ShootAngle = 0; 
    Telemetry.log("distance to hub", distanceToHub);

    Telemetry.log("launch rps", ShootRPS);    

    m_shooter.setShooterSpeed(ShootRPS); //uses rps
    m_kicker.setKickerSpeed(-0.9);
    m_hopper.setHopperSpeed(-0.8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = m_drivetrain.getState().Pose;
    double x = currentPose.getX();
    double y = currentPose.getY();
    double hubX=0;
    double hubY=0;
    if (Alliance.RED == MatchState.getAlliance().get()){
        hubX = 11.901424;
        hubY = 4.021328;
    }
    else {
        hubX = 4.611624;
        hubY = 4.021328;
    }
    Rotation2d heading = currentPose.getRotation();
    double angleToHubNorthRadians = Math.atan2((hubY-y),(hubX-x));
    var alianceColor = MatchState.getAlliance();

    double adjustedShootAngle = 0;
    if (alianceColor.isPresent()){
        if (Alliance.RED == MatchState.getAlliance().get()){
          adjustedShootAngle = angleToHubNorthRadians-heading.getRadians()+Math.PI;
    }
        else {
          adjustedShootAngle = angleToHubNorthRadians-heading.getRadians()+Math.PI;
        }
    }
    

     

    while (adjustedShootAngle<-Math.PI) {
      adjustedShootAngle+=(2*Math.PI);
    }
    while (adjustedShootAngle>Math.PI) {
      adjustedShootAngle-=(2*Math.PI);
    }
     
    Telemetry.log("angle to hub", Math.toDegrees(angleToHubNorthRadians));    

    if (adjustedShootAngle > Math.toRadians(60)){
      adjustedShootAngle=Math.toRadians(60);
    }
    else if (adjustedShootAngle<-Math.toRadians(60)){
      adjustedShootAngle=-Math.toRadians(60);
    }
    
    //turret gear ratio 1:10. 36 degrees per rotation
    m_turret.goToLocation((-adjustedShootAngle+m_turret.returnOffset())/Math.toRadians(36));
    Telemetry.log("adjustedShootAngle", Math.toDegrees(adjustedShootAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
    m_kicker.stopKicker();
    m_hopper.stopHopper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotState.isAutonomous()){
      if (m_timer.get()>=time) {
        return true;
      }
      else {
        return false;
      }
    }
    else{
      return false;
    }
  }
}

/* Math:
 * distance to hub: sqrt((x-a)^2+(y-b)^2), x,y is robot coordinate, a,b is hub coordinate
 * angle to hub: 
 * launch velocity: sqrt[(-gd^2)/((2cos^2(a))(h-dtan(a))] where d is distance to hub, h is height difference, and a is launch angle
 * rotations per min: (60v)/(2pi*r) where r is radius of wheel
 */

