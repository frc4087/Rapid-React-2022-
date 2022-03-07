package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherBase extends SubsystemBase {

  public final WPI_TalonFX rLaunchMotor = new WPI_TalonFX(Constants.RLAUNCH);
  public final WPI_TalonFX lLaunchMotor = new WPI_TalonFX(Constants.LLAUNCH);
  public final PIDController PID = new PIDController(Constants.kLauncherP, Constants.kLauncherI, Constants.kLauncherD);
  //public final SparkMaxPIDController turretPID = launchMotor.getPIDController();
  
  //public DigitalInput hallEffect = new DigitalInput(Constants.HALL);

  public LauncherBase() {
    rLaunchMotor.configFactoryDefault();
    lLaunchMotor.configFactoryDefault();

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

    rLaunchMotor.config_kP(Constants.kLaunchPIDIdx,Constants.kGains_Vel.kP,Constants.kLaunchTimeoutMs);
    lLaunchMotor.config_kP(Constants.kLaunchPIDIdx,Constants.kGains_Vel.kP,Constants.kLaunchTimeoutMs);

    rLaunchMotor.config_kI(Constants.kLaunchPIDIdx,Constants.kGains_Vel.kI,Constants.kLaunchTimeoutMs);
    lLaunchMotor.config_kI(Constants.kLaunchPIDIdx,Constants.kGains_Vel.kI,Constants.kLaunchTimeoutMs);

    rLaunchMotor.config_kD(Constants.kLaunchPIDIdx,Constants.kGains_Vel.kD,Constants.kLaunchTimeoutMs);
    lLaunchMotor.config_kD(Constants.kLaunchPIDIdx,Constants.kGains_Vel.kD,Constants.kLaunchTimeoutMs);

    rLaunchMotor.set(ControlMode.PercentOutput, 0);
    lLaunchMotor.set(ControlMode.PercentOutput, 0);

    rLaunchMotor.setNeutralMode(NeutralMode.Coast);
    lLaunchMotor.setNeutralMode(NeutralMode.Coast);

    rLaunchMotor.setInverted(true);
    lLaunchMotor.setInverted(false);
    // turretPID.setP(Constants.kTurretP);
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

  public void setRPM(double RPM){
    rLaunchMotor.set(ControlMode.Velocity, Constants.rpmToTalonVel(-RPM));
    lLaunchMotor.set(ControlMode.Velocity, Constants.rpmToTalonVel(-RPM));
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