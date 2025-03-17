package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class Constants {

  public static class AlgaePivotConstants {
    //IDs
    public static final int kAlgaePivotID = 14;
    
    //Important Values
    public static final double kTolerance = 0.0;
    
    //Configurations
    public static TalonFXConfiguration AlgaePivotConfiguration = new TalonFXConfiguration();
    public static MotionMagicConfigs AlgaePivotMMConfiguration = new MotionMagicConfigs();

    static {
      AlgaePivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      
      AlgaePivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      AlgaePivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 2.0;

      AlgaePivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      AlgaePivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

      AlgaePivotConfiguration.Slot0.kG = 0.5;
      AlgaePivotConfiguration.Slot0.kS = 0.48046875;
      AlgaePivotConfiguration.Slot0.kV = 0.0; 
      AlgaePivotConfiguration.Slot0.kA = 0.0;
      AlgaePivotConfiguration.Slot0.kP = 1.0;
      AlgaePivotConfiguration.Slot0.kI = 0.0;
      AlgaePivotConfiguration.Slot0.kD = 0.0;

      AlgaePivotConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      AlgaePivotConfiguration.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

      AlgaePivotMMConfiguration.MotionMagicCruiseVelocity = 16.0;
      AlgaePivotMMConfiguration.MotionMagicAcceleration = 256.0;
      AlgaePivotMMConfiguration.MotionMagicExpo_kV = 0.11999999731779099;
      AlgaePivotMMConfiguration.MotionMagicExpo_kA = 0.10000000149011612;
    }
  }

  public static class ElevatorConstants{
    //Motor ID's
    public static final int LIFTID = 15;

    //Sensor ID's
    public static final int UPPERLSID = 1;
    public static final int BOTTOMLSID = 2;

    //PID Constants and Other Important Variables
    public static final double MAXSPEED = 0.5;

    public static final TalonFXConfiguration elevConfigs = new TalonFXConfiguration();
    public static final MotionMagicConfigs elevMMConfigs = new MotionMagicConfigs();
    static {
        elevConfigs.CurrentLimits.StatorCurrentLimit = 120;
        elevConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevConfigs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        elevConfigs.Slot0.kS = 1.75;  
        elevConfigs.Slot0.kG = 2.5;
        elevConfigs.Slot0.kV = 0.0;
        elevConfigs.Slot0.kA = 0.0;  
        elevConfigs.Slot0.kP = 1.0;  
        elevConfigs.Slot0.kI = 0.0;  
        elevConfigs.Slot0.kD = 0.0;  
        elevMMConfigs.MotionMagicCruiseVelocity = 48;    
        elevMMConfigs.MotionMagicAcceleration = 128;
        elevMMConfigs.MotionMagicExpo_kA = 0.10000000149011612;
        elevMMConfigs.MotionMagicExpo_kV =0.11999999731779099; 
    }
  }
}
