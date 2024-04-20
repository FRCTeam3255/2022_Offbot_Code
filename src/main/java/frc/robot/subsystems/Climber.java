// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.preferences.SN_DoublePreference;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constClimber;
import frc.robot.RobotMap.mapClimber;
import frc.robot.RobotPreferences.prefClimber;

public class Climber extends SubsystemBase {

  TalonFX climberMotor;
  TalonFXConfiguration config;

  DigitalInput minSwitch;
  DigitalInput maxSwitch;

  boolean displayOnDashboard;

  public Climber() {
    climberMotor = new TalonFX(mapClimber.CLIMBER_MOTOR_CAN);

    minSwitch = new DigitalInput(mapClimber.CLIMBER_MINIMUM_SWITCH_DIO);
    maxSwitch = new DigitalInput(mapClimber.CLIMBER_MAXIMUM_SWITCH_DIO);

    config = new TalonFXConfiguration();
    configure();

    displayOnDashboard = true;

  }

  public void configure() {
    config.slot0.kP = prefClimber.climberP.getValue();
    config.slot0.kI = prefClimber.climberI.getValue();
    config.slot0.kD = prefClimber.climberD.getValue();

    config.slot0.closedLoopPeakOutput = prefClimber.climberClosedLoopSpeed.getValue();
    config.slot0.allowableClosedloopError = SN_Math.RPMToVelocity(
        prefClimber.climberAllowableClosedLoopError.getValue(), SN_Math.TALONFX_ENCODER_PULSES_PER_COUNT);

    climberMotor.configFactoryDefault();
    climberMotor.configAllSettings(config);

    climberMotor.configReverseSoftLimitEnable(false);
    climberMotor.configForwardSoftLimitEnable(true);
    climberMotor.configForwardSoftLimitThreshold(prefClimber.climberAngledMaxPos.getValue());

    climberMotor.setNeutralMode(NeutralMode.Brake);
    climberMotor.setInverted(constClimber.INVERTED);
  }

  public void setClimberSpeed(double a_speed) {

    double speed = a_speed;

    // cannot ever go below min switch
    if ((isMinSwitch() && speed < 0)) {
      speed = 0;
    }

    // slow down climber speed when switching hooks
    if (getClimberEncoderCounts() > prefClimber.climberSlowdownMinThresholdEncoderCounts.getValue()
        && getClimberEncoderCounts() < prefClimber.climberSlowdownMaxThresholdEncoderCounts.getValue()) {
      speed = speed * prefClimber.climberSlowdownSpeed.getValue();
    }

    // maximum angled position is the physical maximum position, where the max
    // switch and forward soft limit are

    // minimum perpendicular position is the physical minimum position, where the
    // min switch and reverse soft limit are

    climberMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setClimberPosition(SN_DoublePreference a_position) {

    double position = a_position.getValue();

    climberMotor.set(ControlMode.Position, position,
        DemandType.ArbitraryFeedForward, prefClimber.climberArbitraryFeedForward.getValue());
  }

  public double getClimberEncoderCounts() {
    return climberMotor.getSelectedSensorPosition();
  }

  public void resetClimberEncoderCounts() {
    climberMotor.setSelectedSensorPosition(0);
  }

  public void neutralMotorOutput() {
    climberMotor.neutralOutput();
  }

  public boolean isMaxSwitch() {
    return !maxSwitch.get();
  }

  public boolean isMinSwitch() {
    return !minSwitch.get();
  }

  public void displayValuesOnDashboard() {
    displayOnDashboard = true;
  }

  public void hideValuesOnDashboard() {
    displayOnDashboard = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getClimberEncoderCounts() > prefClimber.climberSlowdownMinThresholdEncoderCounts.getValue()
        && getClimberEncoderCounts() < prefClimber.climberSlowdownMaxThresholdEncoderCounts.getValue()) {
      climberMotor.configClosedLoopPeakOutput(0, prefClimber.climberSlowdownSpeed.getValue());
    } else {
      climberMotor.configClosedLoopPeakOutput(0, prefClimber.climberClosedLoopSpeed.getValue());
    }
    if (displayOnDashboard) {
      SmartDashboard.putNumber("Climber Encoder Counts", getClimberEncoderCounts());
      SmartDashboard.putBoolean("Climber Is At Minimum Switch", isMinSwitch());
      SmartDashboard.putBoolean("Climber Is At Maximum Switch", isMaxSwitch());
    }
    if (isMinSwitch()) {
      resetClimberEncoderCounts();
    }

  }
}