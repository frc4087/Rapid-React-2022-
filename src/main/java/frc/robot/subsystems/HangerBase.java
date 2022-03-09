// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerBase extends SubsystemBase {
  /** Creates a new HangerBase. */
  public final CANSparkMax leftHangerMotor = new CANSparkMax(Constants.LHANGER, MotorType.kBrushless);
  public final CANSparkMax rightHangerMotor = new CANSparkMax(Constants.RHANGER, MotorType.kBrushless);
 
  public DoubleSolenoid hangerSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 5); //check these ports

  public final Encoder hangEncoder = new Encoder(Constants.HangEncPort1, Constants.HangEncPort2);
  
  public HangerBase() {
    leftHangerMotor.setIdleMode(IdleMode.kBrake);
    rightHangerMotor.setSmartCurrentLimit(30);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
