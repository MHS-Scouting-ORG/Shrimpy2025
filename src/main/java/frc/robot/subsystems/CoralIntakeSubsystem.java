// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CoralIntakeSubsystem extends SubsystemBase {
  /** Creates a new CoralIntakeSubsystem. */
  private final DigitalInput opticalSensor;
  private final SparkMax coralIntake;
  private final SparkMaxConfig config;

  public CoralIntakeSubsystem(SparkMax intakeMotor) {

    opticalSensor = new DigitalInput(CoralConstants.CORAL_OPTICAL_SENSOR_ID);
    coralIntake = intakeMotor;
    config = new SparkMaxConfig();
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    coralIntake.configure(config, null, PersistMode.kPersistParameters);
    
  }

  // return current value of Optical Switch
  public boolean getOpticalSensor() {
    return opticalSensor.get();
  }

  // set Coral Intake sped to speed
  public void setIntakeSpeed(double speed) {
    coralIntake.set(speed);
  }

  public void stopIntake(){
    coralIntake.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("[C] Optical Sensor", getOpticalSensor());
  }
}
