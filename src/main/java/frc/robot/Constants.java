package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class Constants {
  public static class algaeShooterConstants {
    public static final int algaeShooterID = 16;  // can id for algae shooter 
    public static final double algaeIntakeSpeed = -1; // default -1 for full speed (duty cycle)
    public static final double algaeOutakeSpeed = 0.75; // default 1 for full speed (duty cycle)
    public static final double algaeCurrentThreshold = 45;  //default 20 (amps), increase if triggers unintentionally from motor ramp
    public static final double algaeHoldSpeed = -0.1; //default 0, set negative for active holding (duty cycle)
  }

  public static class AlgaePivotConstants {
    //IDs
    public static final int kAlgaePivotID = 14;
    
    //Important Values
    public static final double kTolerance = 1;
    
    //Configurations
    public static TalonFXConfiguration AlgaePivotConfiguration = new TalonFXConfiguration();
    public static MotionMagicConfigs AlgaePivotMMConfiguration = new MotionMagicConfigs();

    static {
      AlgaePivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      
      AlgaePivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      AlgaePivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 23;

      AlgaePivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      AlgaePivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

      AlgaePivotConfiguration.Slot0.kG = 1.0;
      AlgaePivotConfiguration.Slot0.kS = 0.48046875;
      AlgaePivotConfiguration.Slot0.kV = 0.0; 
      AlgaePivotConfiguration.Slot0.kA = 0.0;
      AlgaePivotConfiguration.Slot0.kP = 1.0;
      AlgaePivotConfiguration.Slot0.kI = 0.0;
      AlgaePivotConfiguration.Slot0.kD = 0.0;

      AlgaePivotConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      AlgaePivotConfiguration.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

      AlgaePivotMMConfiguration.MotionMagicCruiseVelocity = 32.0;
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
      

        elevConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        elevConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 53;

        elevConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        elevConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        elevConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        elevConfigs.CurrentLimits.StatorCurrentLimit = 100;
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
        elevMMConfigs.MotionMagicCruiseVelocity = 120;    
        elevMMConfigs.MotionMagicAcceleration = 160;
        elevMMConfigs.MotionMagicExpo_kA = 0.10000000149011612;
        elevMMConfigs.MotionMagicExpo_kV =0.11999999731779099; 
    }
  }

  public static class CoralConstants{
     // Speeds
    public static final double CORAL_INTAKE_SPEED = 0.8;
    public static final double CORAL_DEPLOY_SPEED = 1;
    public static final double CORAL_OUTTAKE_SPEED = 0;

    public static final double CORAL_PIVOT_UP_SPEED = 0.6;
    public static final double CORAL_PIVOT_DOWN_SPEED = -0.6;

    public static final double HANG_RAISE_SPEED = 0.4;
    public static final double HANG_LOWER_SPEED = -0.4;

    // ID
    public static final int CORAL_INTAKE_ID = 8;
    public static final int CORAL_OPTICAL_SENSOR_ID = 4;
    public static final int CORAL_PIVOT_ID = 7;

    public static final int HANG_ID = 3;

    public static final int HIGHLIMIT_ID = 1;
    public static final int LOWLIMIT_ID = 2;

    // PIVOT PID VALUES

    public static final double kPIVOT_P = 0.1;
    public static final double kPIVOT_I = 0;
    public static final double kPIVOT_D = 0;
    }
}
