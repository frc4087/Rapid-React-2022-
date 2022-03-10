package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeBase extends SubsystemBase {

  public final CANSparkMax IntakeMotor = new CANSparkMax(Constants.INTAKE, MotorType.kBrushless);

  public DoubleSolenoid intakeSol1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 7);
  //public DoubleSolenoid intakeSol2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 6);

  public IntakeBase() {
    IntakeMotor.setSmartCurrentLimit(30);
   
  }

  public void setIntake(double velocity){
    IntakeMotor.set(velocity);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

}