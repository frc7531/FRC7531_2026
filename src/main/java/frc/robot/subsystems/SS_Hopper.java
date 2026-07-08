// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Hopper extends SubsystemBase {
  /** Creates a new SS_Hopper. */
  
  public TalonFX hotDogRoller = new TalonFX(21, "rio");
  public TalonFX m_carWash = new TalonFX(22, "CANivore");

  public SS_Hopper() {
    //setDefaultCommand(new RunCommand(() -> hotDogRollersOff(), this));
    
  }


  public void hotDogRollersOn() {
    hotDogRoller.set(-1);
  }

  public void hotDogRollersBack()
  {
    hotDogRoller.set(.8);
  }
  
  public void hotDogRollersOff() {
    hotDogRoller.set(0);
  }

  public void carWashOn()
  {
    m_carWash.set(-1);
  }

  public void carWashBack()
  {
    m_carWash.set(.5);
  }

  public void carWashOff()
  {
    m_carWash.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
