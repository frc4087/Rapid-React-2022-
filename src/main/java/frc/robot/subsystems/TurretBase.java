package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretBase extends SubsystemBase {

  public final CANSparkMax turretMotor = new CANSparkMax(Constants.TURR, MotorType.kBrushless);
  public final SparkMaxPIDController turretPID = turretMotor.getPIDController();
  
  public DigitalInput hallEffect = new DigitalInput(Constants.HALL);

  public TurretBase() {

    turretMotor.setSmartCurrentLimit(20);

    turretMotor.setIdleMode(IdleMode.kBrake);
    turretPID.setP(Constants.kTurretP);
    turretPID.setI(Constants.kTurretI);
    turretPID.setD(Constants.kTurretD);
    turretPID.setFF(Constants.kTurretFF);
    turretPID.setIZone(Constants.kTurretIZ);
    turretPID.setOutputRange(-0.25, 0.25);

    turretMotor.getEncoder().setPositionConversionFactor(Constants.turretPosConFac); // 46.67 motor rotations is one turret
                                                                             // rotation
    turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kForward, 95); //10
    turretMotor.setSoftLimit(SoftLimitDirection.kReverse, -95); //-90
  }

  public boolean getHallEffect(){
    return !hallEffect.get();
  }

  public void resetEncoder(){
    if (getHallEffect()){
      turretMotor.getEncoder().setPosition(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPos(double setpoint) {
    turretPID.setReference(setpoint, ControlType.kPosition);
  }
}