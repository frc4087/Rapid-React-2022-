// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerBase extends SubsystemBase {
  /** Creates a new HangerBase. */
  public final CANSparkMax leftHangerMotor = new CANSparkMax(Constants.LHANGER, MotorType.kBrushless);
  public final CANSparkMax rightHangerMotor = new CANSparkMax(Constants.RHANGER, MotorType.kBrushless);
  public final MotorControllerGroup hangerMotors = new MotorControllerGroup(leftHangerMotor, rightHangerMotor);
 
  public DoubleSolenoid hangerSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 5); //check these ports

  public final Encoder hangEncoder = new Encoder(Constants.HangEncPort1, Constants.HangEncPort2);
  
  public HangerBase() {
    leftHangerMotor.setIdleMode(IdleMode.kBrake);
    rightHangerMotor.setIdleMode(IdleMode.kBrake);
    
    leftHangerMotor.setInverted(true);
    rightHangerMotor.setInverted(false);
    
    rightHangerMotor.setSmartCurrentLimit(40);
    leftHangerMotor.setSmartCurrentLimit(40);
    
    //leftHangerMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // leftHangerMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // leftHangerMotor.setSoftLimit(SoftLimitDirection.kForward, 100);
    // leftHangerMotor.setSoftLimit(SoftLimitDirection.kReverse, -100);
    //rightHangerMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // rightHangerMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // rightHangerMotor.setSoftLimit(SoftLimitDirection.kForward, 100);
    // rightHangerMotor.setSoftLimit(SoftLimitDirection.kReverse, -100);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
