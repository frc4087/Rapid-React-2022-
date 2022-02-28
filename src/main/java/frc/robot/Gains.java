package frc.robot;

public class Gains{
    public final double kP,
                        kI,
                        kD,
                        kF;
    public final int kIzone;
    public final double kPeakOutput;

    public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput){
        kP = _kP;
        kI = _kI;
        kD = _kD;
        kF = _kF;
        kIzone = _kIzone;
        kPeakOutput = _kPeakOutput;
    }
}