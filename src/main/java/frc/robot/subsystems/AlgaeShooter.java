// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// neo 550 for algea intake
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.algaeShooterConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

 
public class AlgaeShooter extends SubsystemBase {
    
    private final TalonFX algaeMotor;
    public boolean algaeHeld;
    public boolean algaeIntakeQuery;

  public AlgaeShooter() {
    algaeMotor = new TalonFX(algaeShooterConstants.algaeShooterID);
    algaeHeld = false;
    algaeIntakeQuery = false;
  }

  public void algaeIntake() {
    algaeMotor.setNeutralMode(NeutralModeValue.Coast);
    algaeHeld = false;
    algaeIntakeQuery = true;
      if (!algaeHeld) {
          algaeMotor.set(algaeShooterConstants.algaeIntakeSpeed); // Full speed intake
      }
  }

  public void checkMotorStall() {
      if (algaeMotor.getStatorCurrent().getValueAsDouble() > algaeShooterConstants.algaeCurrentThreshold && algaeMotor.get() < -0.1) {  //check the direction )    // replace with .getOutPutCurrent for sparks
          algaeHeld = true; // Global variable for compatability
          idleHold();
      }
  }
  private void idleHold() {
      if (algaeHeld) {
          algaeMotor.set(algaeShooterConstants.algaeHoldSpeed); // Hold the algae
          algaeMotor.setNeutralMode(NeutralModeValue.Brake);
      }
  }
  public void stopIntake() {
        algaeMotor.set(0); // Stop the motor
        algaeMotor.setNeutralMode(NeutralModeValue.Coast);
        algaeHeld = false;
        algaeIntakeQuery = false;

  }
  public void algaeOutake() {
        algaeHeld = false;
        algaeIntakeQuery = false;
        algaeMotor.setNeutralMode(NeutralModeValue.Coast);
        algaeMotor.set(algaeShooterConstants.algaeOutakeSpeed); // Reverse to shoot the algae
  }

  @Override
  public void periodic() {
    if (algaeIntakeQuery) {
        checkMotorStall(); // Check if the motor is stalled
    }
      
      SmartDashboard.putBoolean("algaeHeld", algaeHeld); // Display if the algae is held
      SmartDashboard.putNumber("current", algaeMotor.getStatorCurrent().getValueAsDouble()); // replace with .getOutPutCurrent for sparks
  }
}