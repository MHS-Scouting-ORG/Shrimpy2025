package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralPivotSubsystem extends SubsystemBase {

  private final RelativeEncoder coralPivotEnc;
  private final SparkMax coralPivot;
  private final SparkMaxConfig config;
  private final SparkLimitSwitch ls;
  private final PIDController coralPivotPidController;
  private boolean pidStat;
  private double prevError, command;

  public CoralPivotSubsystem(SparkLimitSwitch limitSwitch, SparkMax pivotMotor) {
    coralPivot = pivotMotor;
    coralPivotEnc = coralPivot.getEncoder();
    ls = limitSwitch;
    prevError = 0;
    command = 0;
    pidStat = false;

    config = new SparkMaxConfig();
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    coralPivot.configure(config, null, null);

    coralPivotPidController = new PIDController(CoralConstants.kPIVOT_P, CoralConstants.kPIVOT_I, CoralConstants.kPIVOT_D);

  }

  public boolean getPivotLimitSwitch() {
    return ls.isPressed();
  }

  public void setCoralPivotSetpoint(double point) {
    coralPivotPidController.setSetpoint(point);
  }

  public double getCoralPivotSetpoint() {
    return coralPivotPidController.getSetpoint();
  }

  public boolean atSetpoint() {
    return coralPivotPidController.atSetpoint();
  }

  public double getCoralPivotEnc() {
    return coralPivotEnc.getPosition();
  }

  public void resetCoralPivotEnc() {
    coralPivotEnc.setPosition(0);
  }

  public void setCoralPivotSpeed(double val) {
    coralPivot.set(val);
  }

  public void stop() {
    coralPivot.stopMotor();
  }

  public void setPIDStat(boolean stat) {
    pidStat = stat;
  }

  @Override
  public void periodic() {

    double currError = getCoralPivotSetpoint() - getCoralPivotEnc();

    if (pidStat) {
      command = coralPivotPidController.calculate(getCoralPivotEnc(), getCoralPivotSetpoint());
      if (command > CoralConstants.CORAL_PIVOT_UP_SPEED) {
        command = CoralConstants.CORAL_PIVOT_UP_SPEED;
      } else if (command < CoralConstants.CORAL_PIVOT_DOWN_SPEED) {
        command = CoralConstants.CORAL_PIVOT_DOWN_SPEED;
      }

      if (currError < 0 && prevError > 0) {
        coralPivotPidController.reset();
      } else if (currError > 0 && prevError < 0) {
        coralPivotPidController.reset();
      }

      prevError = currError;

    }else{
      command = 0;
    }

    if (getPivotLimitSwitch()) {
      resetCoralPivotEnc();
      if (command < 0) {
        command = 0;
      }
    }

    setCoralPivotSpeed(command);

    SmartDashboard.putNumber("[C] Pivot Position", getCoralPivotEnc());
    SmartDashboard.putNumber("[C] Pivot PID setpoint", getCoralPivotSetpoint());
    SmartDashboard.putNumber("[C] Command Output", command);
    SmartDashboard.putBoolean("[C] Pivot LS", getPivotLimitSwitch());
    SmartDashboard.putBoolean("[C] At Setpoint", atSetpoint());
    // This method will be called once per scheduler run
  }
}
