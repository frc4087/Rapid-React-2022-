// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimeLightBase;
import frc.robot.subsystems.TurretBase;

public class Tracking extends CommandBase {
  /** Creates a new Tracking. */
  public LimeLightBase limelight = new LimeLightBase();
  public static TurretBase turret = new TurretBase();
  boolean tracking = false;

  public Tracking() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public void setTracking(boolean tracking) {
    // Robot.m_robotContainer.m_limeLightBase.m_limelightTable.getEntry("camMode").setNumber(tracking ? 0 : 1);
    // Robot.m_robotContainer.m_limeLightBase.m_limelightTable.getEntry("ledMode").setNumber(tracking ? 0 : 1);
  }
  



  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      limelight.setTracking(tracking);
      if (tracking) {
        RobotContainer.setpoint = -turret.turretMotor.getEncoder().getPosition()
            + (limelight.get("tx"));
            // + (1.25 / 154) * Robot.m_robotContainer.launchCommand.launcher.lidar.getDistance());
        turret.turretPID.setI(0.000025);
      }
      turret.turretPID.setI(0.00000);
      turret.setPos(RobotContainer.setpoint);
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
