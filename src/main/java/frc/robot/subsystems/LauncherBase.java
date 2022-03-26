package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherBase extends SubsystemBase {

  public final WPI_TalonFX rLaunchMotor = new WPI_TalonFX(Constants.RLAUNCH);
  public final WPI_TalonFX lLaunchMotor = new WPI_TalonFX(Constants.LLAUNCH);
  public final CANSparkMax blaunchMotor = new CANSparkMax(Constants.BLAUNCH, MotorType.kBrushless);
  //public final PIDController PID = new PIDController(Constants.kLauncherP, Constants.kLauncherI, Constants.kLauncherD);
  public final SparkMaxPIDController BackPID = blaunchMotor.getPIDController();
  //public final SparkMaxPIDController turretPID = launchMotor.getPIDController();
  
  //public DigitalInput hallEffect = new DigitalInput(Constants.HALL);

  public LauncherBase() {
    rLaunchMotor.configFactoryDefault();
    lLaunchMotor.configFactoryDefault();
    blaunchMotor.getEncoder().setVelocityConversionFactor(Constants.backLaunchPosConFac);
    blaunchMotor.setIdleMode(IdleMode.kCoast);
    BackPID.setP(Constants.kBackP);
    BackPID.setI(Constants.kBackI);
    BackPID.setD(Constants.kBackD);
    BackPID.setFF(Constants.kBackFF);
    BackPID.setIZone(Constants.kBackIZ);
    BackPID.setOutputRange(-1, 1);
    

    // rLaunchMotor.configNeutralDeadband(0.001);
    // lLaunchMotor.configNeutralDeadband(0.001);

    rLaunchMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,Constants.kLaunchPIDIdx,Constants.kLaunchTimeoutMs);
    lLaunchMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,Constants.kLaunchPIDIdx,Constants.kLaunchTimeoutMs);

    // rLaunchMotor.configNominalOutputForward(0,Constants.kLaunchTimeoutMs);
    // lLaunchMotor.configNominalOutputForward(0,Constants.kLaunchTimeoutMs);

    // rLaunchMotor.configNominalOutputReverse(0,Constants.kLaunchTimeoutMs);
    // lLaunchMotor.configNominalOutputReverse(0,Constants.kLaunchTimeoutMs);

    // rLaunchMotor.configPeakOutputForward(1,Constants.kLaunchTimeoutMs);
    // lLaunchMotor.configPeakOutputForward(1,Constants.kLaunchTimeoutMs);

    // rLaunchMotor.configPeakOutputReverse(-1,Constants.kLaunchTimeoutMs);
    // lLaunchMotor.configPeakOutputReverse(-1,Constants.kLaunchTimeoutMs);

    rLaunchMotor.config_kF(Constants.kLaunchPIDIdx,Constants.kGains_Vel.kF,Constants.kLaunchTimeoutMs);
    lLaunchMotor.config_kF(Constants.kLaunchPIDIdx,Constants.kGains_Vel.kF,Constants.kLaunchTimeoutMs);

    rLaunchMotor.config_kP(Constants.kLaunchPIDIdx,Constants.kLauncherP,Constants.kLaunchTimeoutMs);
    lLaunchMotor.config_kP(Constants.kLaunchPIDIdx,Constants.kLauncherP,Constants.kLaunchTimeoutMs);

    rLaunchMotor.config_kI(Constants.kLaunchPIDIdx,Constants.kLauncherI,Constants.kLaunchTimeoutMs);
    lLaunchMotor.config_kI(Constants.kLaunchPIDIdx,Constants.kLauncherI,Constants.kLaunchTimeoutMs);

    rLaunchMotor.config_kD(Constants.kLaunchPIDIdx,Constants.kLauncherD,Constants.kLaunchTimeoutMs);
    lLaunchMotor.config_kD(Constants.kLaunchPIDIdx,Constants.kLauncherD,Constants.kLaunchTimeoutMs);

    rLaunchMotor.set(ControlMode.Velocity, 0);
    lLaunchMotor.set(ControlMode.Velocity, 0);

    rLaunchMotor.setNeutralMode(NeutralMode.Coast);
    lLaunchMotor.setNeutralMode(NeutralMode.Coast);

    rLaunchMotor.setInverted(true);
    lLaunchMotor.setInverted(false);
    // BackPID.setP(Constants.kTurretP);
    // turretPID.setI(Constants.kTurretI);
    // turretPID.setD(Constants.kTurretD);
    // turretPID.setFF(Constants.kTurretFF);
    // turretPID.setIZone(Constants.kTurretIZ);
    // turretPID.setOutputRange(-0.25, 0.25);
    

    // turretMotor.getEncoder().setPositionConversionFactor((1 / 46.67) * 360); // 46.67 motor rotations is one turret
                                                                             // rotation
    // turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // turretMotor.setSoftLimit(SoftLimitDirection.kForward, 90);
    // turretMotor.setSoftLimit(SoftLimitDirection.kReverse, -90);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setFrontRPM(double RPM){
    rLaunchMotor.set(ControlMode.Velocity, Constants.rpmToTalonVel(-2*RPM));
    lLaunchMotor.set(ControlMode.Velocity, Constants.rpmToTalonVel(-2*RPM));
    //BackPID.setReference(RPM, ControlType.kVelocity);
  }

  public void setBackRPM(double RPM){
    // rLaunchMotor.set(ControlMode.Velocity, Constants.rpmToTalonVel(-RPM));
    // lLaunchMotor.set(ControlMode.Velocity, Constants.rpmToTalonVel(-RPM));
    BackPID.setReference(16*RPM, ControlType.kVelocity);
  }


  public void setPO(double PO){
    rLaunchMotor.set(ControlMode.PercentOutput, Constants.rpmToTalonVel(-PO));
    lLaunchMotor.set(ControlMode.PercentOutput, Constants.rpmToTalonVel(-PO));
    blaunchMotor.set(PO*4.0);
  }

  public double getRPM(){
    double r = rLaunchMotor.getSelectedSensorVelocity();
    //double l = Constants.talonVelToRPM(lLaunchMotor.getSelectedSensorVelocity());
    return Math.abs(r);
  }

  // public void setPos(double setpoint) {
  //   turretPID.setReference(-setpoint, ControlType.kPosition);
  // }
}