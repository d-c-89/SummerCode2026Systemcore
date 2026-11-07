// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// 你好。你可以读中文吗？我们是三四0八。

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import org.wpilib.vision.stream.CameraServer;
import org.wpilib.vision.camera.HttpCamera;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.TimedRobot;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.telemetry.*;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import frc.robot.LimelightHelpers.PoseEstimate;
import org.wpilib.math.util.Units;
import frc.robot.subsystems.TurretSubsystem;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    TurretSubsystem m_turret = new TurretSubsystem();

    private final RobotContainer m_robotContainer;
    
    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 

        LimelightHelpers.SetIMUMode("limelight", 0);
        // Updating vision with limelight
        var driveState = m_robotContainer.drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omega);  

       
        //use tag filtering to decide whether to use mt1 or mt2

        LimelightHelpers.SetRobotOrientation("limelight", headingDeg, omegaRps*360, 0, 0, 0, 0); // Changed limelight orientation here. 
        PoseEstimate llMeasurement;
        if (RobotState.isAutonomous())
        {
            llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            //llMeasurement= LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        }
        else 
        {
            llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        }
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
            m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        }


        

    }

    @Override
    public void disabledInit() {
        LimelightHelpers.SetIMUMode("limelight", 1);
        
    }

    @Override
    public void disabledPeriodic() {}

    @Override   
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        
        HttpCamera limelightFeed = new HttpCamera("Limelight Stream", "http://limelight.local:5800/stream.mjpg");
        
        CameraServer.startAutomaticCapture(limelightFeed);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        HttpCamera limelightFeed = new HttpCamera("Limelight Stream", "http://limelight.local:5800/stream.mjpg");
        
        CameraServer.startAutomaticCapture(limelightFeed);
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void utilityInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void utilityPeriodic() {}

    @Override
    public void utilityExit() {}

    @Override
    public void simulationPeriodic() {}
}
