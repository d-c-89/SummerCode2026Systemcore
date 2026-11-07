// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CAN;
import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants.operatorConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import org.wpilib.hardware.bus.CANPort;

public class HopperSubsystem extends SubsystemBase 
{
  
    private final SparkMax m_HopperMotor;
  /** Creates a new ExampleSubsystem. */

  
  // setup hopper subsystem
  public HopperSubsystem()
   {
    
    m_HopperMotor = new SparkMax(CANPort.CAN_S0, operatorConstants.kHopperMotorId, MotorType.kBrushless);
   //brushed or brushless???
    

  }
  public void setHopperSpeed(double speed)

  {
    m_HopperMotor.setThrottle(speed);
  }

  public void stopHopper()
  {
    m_HopperMotor.setThrottle(0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command runHopper()
  {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return this.startEnd(
      ()->{
        setHopperSpeed(operatorConstants.kHopperSpeed);
      },
      () -> {
        stopHopper();
      }

    );
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
