// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class RobotMap {

  public static final class mapClimber {

    public static final int CLIMBER_MOTOR_CAN = 10;
    public static final int CLIMBER_MINIMUM_SWITCH_DIO = 6;
    public static final int CLIMBER_MAXIMUM_SWITCH_DIO = 9;
    // TODO: Physically Plug in Climber Limit Switch
  }

  public static final class mapControllers {

    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;

  }

  public static final class mapDrivetrain {

    public static final int LEFT_LEAD_MOTOR_CAN = 20;
    public static final int LEFT_FOLLOW_MOTOR_CAN = 21;
    public static final int RIGHT_LEAD_MOTOR_CAN = 22;
    public static final int RIGHT_FOLLOW_MOTOR_CAN = 23;

  }

  public static final class mapHood {

    public static final int HOOD_MOTOR_CAN = 25;

    public static final int HOOD_BOTTOM_SWITCH_DIO = 1;

  }

  public static final class mapIntake {

    public static final int INTAKE_MOTOR_CAN = 30;

  }

  public static final class mapShooter {

    public static final int LEAD_MOTOR_CAN = 40;
    public static final int FOLLOW_MOTOR_CAN = 41;

  }

  public static final class mapTransfer {

    public static final int ENTRANCE_MOTOR_CAN = 50;
    public static final int BOTTOM_MOTOR_CAN = 51;
    public static final int TOP_MOTOR_CAN = 52;

    public static final int TOP_SWITCH_DIO = 2;
    public static final int BOTTOM_SWITCH_DIO = 0;

  }

  public static final class mapTurret {

    public static final int TURRET_MOTOR_CAN = 60;

  }

}
