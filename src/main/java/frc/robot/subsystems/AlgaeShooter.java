// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// neo 550 for algae intake
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
 
public class AlgaeShooter extends SubsystemBase {
  private final TalonFX algaeMotor = new TalonFX(16); 
  public boolean ballHeld = false;

  

  public void algaeIntake() {
    algaeMotor.setNeutralMode(NeutralModeValue.Coast);
    TalonFXConfiguration algaeConfig = new TalonFXConfiguration();
    algaeConfig.CurrentLimits.StatorCurrentLimit = 120;
    ballHeld = false;
      if (!ballHeld) {
          algaeMotor.set(-1); // Full speed intake
      }
  }

//   public void checkMotorStall() {
//       if (algaeMotor.getStatorCurrent().getValueAsDouble() > 20 && algaeMotor.get() < -0.08) {  //check the direction )    // replace with .getOutPutCurrent for sparks
//           algaeMotor.set(0); // Stop the motor
//           algaeMotor.setNeutralMode(NeutralModeValue.Brake);
//           ballHeld = true;
//       } else {
//           ballHeld = false;
//           algaeMotor.setNeutralMode(NeutralModeValue.Coast);
//       }
//   }

public boolean checkMotorStall() {
    if (algaeMotor.getStatorCurrent().getValueAsDouble() > 30 && algaeMotor.get() < -0.08) {  //check the direction )    // replace with .getOutPutCurrent for sparks
        algaeMotor.set(0); // Stop the motor
        algaeMotor.setNeutralMode(NeutralModeValue.Brake);
        ballHeld = true;
        return true;
    } else {
        ballHeld = false;
        algaeMotor.setNeutralMode(NeutralModeValue.Coast);
        return false;
    }
}

  public void stopIntake() {
      algaeMotor.set(0); // Stop the motor
  }
  public void algaeOutake() {
    
        ballHeld = false;
        algaeMotor.set(1); // Reverse to shoot the algae
      

  }

  public void idle() {
    algaeMotor.set(0.2);
  }

  @Override
  public void periodic() {
    //   checkMotorStall(); // Check if the motor is stalled
    
      SmartDashboard.putBoolean("ballHeld", ballHeld); // Display if the ball is held
      SmartDashboard.putNumber("current", algaeMotor.getStatorCurrent().getValueAsDouble()); // replace with .getOutPutCurrent for sparks
  }
}