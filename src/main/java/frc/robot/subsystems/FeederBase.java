// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederBase extends SubsystemBase {

public final CANSparkMax BottomFeederMotor = new CANSparkMax(Constants.BOTTOMFEED, MotorType.kBrushless);
public final CANSparkMax TopFeederMotor = new CANSparkMax(Constants.TOPFEED, MotorType.kBrushless);
public final DoubleSolenoid TopFeederBreak = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 6);

  
  public FeederBase () {
    TopFeederMotor.setIdleMode(IdleMode.kBrake);

    TopFeederMotor.setSmartCurrentLimit(20);
    BottomFeederMotor.setSmartCurrentLimit(30);
  }
  

  @Override
  public void periodic() {
    if(TopFeederMotor.get()==0)
      TopFeederBreak.set(Value.kForward);
    else{
      TopFeederBreak.set(Value.kReverse);
    }
  }

  public void BottomFeederSet(double velocity){
    BottomFeederMotor.set(velocity);
  }
  
  public void TopFeederSet(double velocity){
    BottomFeederMotor.set(velocity);
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
