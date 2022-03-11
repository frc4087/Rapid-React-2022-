// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //JOYSTICKS -------------------------------------------------------------------------------------------------------------------------------
    
    public static final int XL = 0,
                            YL = 1,
                            XR = 4,
                            YR = 5;

    public static final int kLeftBumper = 5,
                            kRightBumper = 6,
                            kLeftStick = 9,
                            kRightStick = 10,
                            kA = 1,
                            kB = 2,
                            kX = 3,
                            kY = 4,
                            kBack = 7,
                            kStart = 8;

    //DRIVEBASE -------------------------------------------------------------------------------------------------------------------------------
    
    //Talons
    public final static int L1=2, 
                            L2=3, 
                            L3=4, 
                            R1=5, 
                            R2=6, 
                            R3=7;

    //Drive speed limiters
    public static final double SpeedDivider = 10.0,
                               secondsForOpenRamp = 0.8,
                               CurrentLimmit = 30;

    public final static int kDriveTimeoutMs = 30,
                            kDrivePIDIdx = 0;
    //INTAKE ----------------------------------------------------------------------------------------------------------------------------------
    
    //Sparks
    public static final int INTAKE = 7;

    //Speeds
    public static final double IMSpeed = 1.0;

    //FEEDER ----------------------------------------------------------------------------------------------------------------------------------

    //Sparks
    public final static int BOTTOMFEED = 10,
                            TOPFEED = 1;
                            
    //Motor Speeds
    public static final double  BFMSpeed = 1.0,
                                TFMSpeed = 0.8;

    //TURRET ----------------------------------------------------------------------------------------------------------------------------------

    //Sparks
    public final static int TURR = 6;

    //PID
    public final static double  kTurretP = 0.1,
                                kTurretI = 0.0,
                                kTurretD = 0.0,
                                kTurretFF = 0.0,
                                kTurretIZ = 0.0;

    public final static double turretPosConFac = ((1/70.0)*360);

    //LAUNCHER --------------------------------------------------------------------------------------------------------------------------------
    
    //Sparks
    public final static int RLAUNCH = 0,
                            LLAUNCH = 1;
    
    //Config
    public final static int kLaunchTimeoutMs = 30,
                            kLaunchPIDIdx = 0;

    //PID
    public final static double  kLauncherP = 0,// .0000035,
                                kLauncherI = 0.000000002,
                                kLauncherD = 0,
                                kLauncherFF = .00017647,
                                kLauncherIZ = 0.0;

    //Limelight
    public static final double  heightOfLimelight = 21.5/12, //height limelight is mounted at in feet
                                heightUpper = 10, //TO BE CHANGED from 10!!! //height vision target at upper port is mounted at in feet
                                heightLower = 41.0/12.0, //TO BE CHANGED from 4!!! //height vision target at lower port is mounted at in feet
                                angleOfCamera = 27; //angle limelight is mounted at relative to verticle in degrees, 28.8 was 

    
    /**
     * Converts the RPM to the speed readable by the talons
     *
     * @param RPM The speed of the object in rotations per minute
     * 
     * @return The velocity of that the Talons understands that corresponds to the given RPM
     */
    public static double rpmToTalonVel(double RPM){
        if(Math.abs(RPM) < 5000){
            return RPM*(2048.0/600.0) *(24.0/36.0);//look into this
        } else{
            return 0.0;
        }
    }
    /**
     * Converts the speed readable by the talons to the RPM
     *
     * @param vel The speed of the object in rotations per minute
     * 
     * @return The velocity of that the Talons understands that corresponds to the given RPM
     */
    public static double talonVelToRPM(double vel){
        return vel / ((2048.0/600.0) * (24.0/36.0));
    }
    
    public final static Gains kGains_Vel = new Gains (0.03, 0, 0, 0, 0, 0);//0.3,0.0006,0.2, 0, 0, 0);
      
    //HANGER----------------------------------------------------------------------------------------------------------------------------------

    public final static int LHANGER = 11,
                            RHANGER = 12; //Fix this later 

    public final static int HangEncPort1 = 1,
                            HangEncPort2 = 2; //change


    //SENSORS --------------------------------------------------------------------------------------------------------------------------------
    
    public final static int HALL = 9, //hall effects
                            bean = 0, //beambreak
                            blink = 4; //blinkin
    
    public static final double  autoIdle = 0.27, //blinkin color palettes
                                teleOpIdle = 0.13,
                                strobeGold = -0.07,
                                green = 0.77,
                                blue = 0.87,
                                red = 0.61,
                                violet = 0.91;
            
    //AUTO -----------------------------------------------------------------------------------------------------------------------------------
    
    // Characterization Toolsuite Constants
    public static final double ksVolts = 0.205,//0.65634, 
                               kvVoltSecondsPerMeter = 2.1,//0.1106, 
                               kaVoltSecondsSquaredPerMeter = 0.217,//0.095387,
                               kTrackwidthMeters = 0.635,
                               kP = 1,//0.17833, 
                               kD = 0.0, 
                               kMaxSpeedMetersPerSecond = 4.953,
                               kMaxAccelerationMetersPerSecondSquared = 8.5344,
                               kRamseteB = 2, 
                               kRamseteZeta = 0.7;
    
    public static final DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kEncoderDistancePerPulse = (4 * Math.PI * 2.54 * 9) / (100.0 * 2048 * 64); //gear ratio is 64 to 9
 
    public static final boolean kGyroReversed = false;



}
