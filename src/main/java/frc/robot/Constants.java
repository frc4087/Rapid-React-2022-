// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
   
    //Limelight
    public static final double  heightOfLimelight = 21.5/12, //height limelight is mounted at in feet
                                heightUpper = 10, //TO BE CHANGED from 10!!! //height vision target at upper port is mounted at in feet
                                heightLower = 41.0/12.0, //TO BE CHANGED from 4!!! //height vision target at lower port is mounted at in feet
                                angleOfCamera = 27; //angle limelight is mounted at relative to verticle in degrees, 28.8 was 

    //Motor speeds, from 0 to 1
    public static final double  BFMSpeed = 1.0,
                                TFMSpeed = 0.8,
                                IMSpeed = 1.0;

    // Motor Controllers

        //Spark Maxs
    public final static int TURR = 6,
                            BOTTOMFEED = 10,
                            TOPFEED = 1,
                            INTAKE = 7;
     
        // Talons
    public final static int RLAUNCH = 0,
                            LLAUNCH = 1,
                            L1=2, 
                            L2=3, 
                            L3=4, 
                            R1=5, 
                            R2=6, 
                            R3=7;

    public final static int kTimeoutMs = 30,
                            kPIDloopIdx = 0;
    // Launcher PID
    public final static double  kLauncherP = 0,// .0000035,
                                kLauncherI = 0.000000002,
                                kLauncherD = 0,
                                kLauncherFF = .00017647,
                                kLauncherIZ = 0.0;

    // Turret PID
    public final static double  kTurretP = 0.00725,
                                kTurretI = 0.000025,
                                kTurretD = 0.00,
                                kTurretFF = 0.0,
                                kTurretIZ = 0.0;

    public final static double turretPosConFac = ((1/98.0)*360);
    
    public static double rpmToTalonVel(double RPM){
        if(Math.abs(RPM) < 5000){
            return RPM*(2048.0/600.0) *(24.0/36.0);
        } else{
            return 0.0;
        }
    }

    public static double talonVelToRPM(double vel){
        return vel / ((2048.0/600.0) * (24.0/36.0));
    }
    
    public final static Gains kGains_Vel = new Gains (0.3,0.0006,0.2, 0, 0, 0);
                        

    // Sensors
    public final static int HALL = 9, //hall effects
                            bean = 0, //beambreak
                            blink = 7; //blinkin
    
    // Joystick Axes
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

    //drive speed controller
    public static final double SpeedDivider = 10.0,
                               secondsForOpenRamp = 0.8,
                               CurrentLimmit = 30;
}
