// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




//SET THE LIMELIGHT AT 28 or 27 DEGRESS TO THE HORIZONTAL FOR THE FIRST LIMELIGHT POSITION**





package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;

public class LimeLightBase extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = m_limelightTable.getEntry("tx");
  NetworkTableEntry ty = m_limelightTable.getEntry("ty");
  //NetworkTableEntry ta = m_limelightTable.getEntry("ta");
  public double x;
  public double y;
  public static double conversionFac = 3.5/2.7;

  public LimeLightBase() {
    m_limelightTable.getEntry("pipeline").setNumber(0);
    m_limelightTable.getEntry("camMode").setNumber(1);
    m_limelightTable.getEntry("ledMode").setNumber(1);
  }
  
  public double get(String var) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(var).getDouble(0.0);
  }

  public void setTracking(boolean tracking) {
    m_limelightTable.getEntry("camMode").setNumber(tracking ? 0 : 1);
    m_limelightTable.getEntry("ledMode").setNumber(tracking ? 0 : 1);
  }
  
  public double getTx(){
    return tx.getDouble(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    //double distance = (Constants.heightLower-Constants.heightOfLimelight)/Math.tan(Math.toRadians(Constants.angleOfCamera + Math.atan(y)));;
    //double distance = conversionFac*(Constants.heightLower-Constants.heightOfLimelight)/Math.tan(Math.toRadians(y + Constants.angleOfCamera));

    //post to smart dashboard periodically
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("Distance Inches", distance*12);
  }

  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
