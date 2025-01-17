package frc.robot;

public class Schematic {
  // Drive (CAN IDs)
  public static final int pigeonCanId;

  public static final int frontLeftDriveCanId;
  public static final int backLeftDriveCanId;
  public static final int frontRightDriveCanId;
  public static final int backRightDriveCanId;

  public static final int frontLeftTurnCanId;
  public static final int backLeftTurnCanId;
  public static final int frontRightTurnCanId;
  public static final int backRightTurnCanId;

  public static final int frontLeftAbsoluteEncoderCanId;
  public static final int backLeftAbsoluteEncoderCanId;
  public static final int frontRightAbsoluteEncoderCanId;
  public static final int backRightAbsoluteEncoderCanId;

  // Algae Effector (Motor IDs)
  public static final int PIVOT_ID;
  public static final int REMOVAL_ID;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2024_COMP -> {
        // Drive (CAN IDs)
        pigeonCanId = 9;

        frontLeftDriveCanId = 1;
        backLeftDriveCanId = 3;
        frontRightDriveCanId = 5;
        backRightDriveCanId = 7;

        frontLeftTurnCanId = 2;
        backLeftTurnCanId = 4;
        frontRightTurnCanId = 6;
        backRightTurnCanId = 8;

        frontLeftAbsoluteEncoderCanId = 10;
        backLeftAbsoluteEncoderCanId = 11;
        frontRightAbsoluteEncoderCanId = 12;
        backRightAbsoluteEncoderCanId = 13;

        PIVOT_ID = 0;
        REMOVAL_ID = 0;
      }
      case ROBOT_2025_COMP -> {
        // TODO: Change for 2025 as needed
        // Drive (CAN IDs)
        pigeonCanId = 9;

        frontLeftDriveCanId = 1;
        backLeftDriveCanId = 3;
        frontRightDriveCanId = 5;
        backRightDriveCanId = 7;

        frontLeftTurnCanId = 2;
        backLeftTurnCanId = 4;
        frontRightTurnCanId = 6;
        backRightTurnCanId = 8;

        frontLeftAbsoluteEncoderCanId = 10;
        backLeftAbsoluteEncoderCanId = 11;
        frontRightAbsoluteEncoderCanId = 12;
        backRightAbsoluteEncoderCanId = 13;

        // Algae Effector (Motor IDs)
        PIVOT_ID = 1;
        REMOVAL_ID = 2;
      }

      default -> {
        // Drive (CAN IDs)
        pigeonCanId = 0;

        frontLeftDriveCanId = 0;
        backLeftDriveCanId = 0;
        frontRightDriveCanId = 0;
        backRightDriveCanId = 0;

        frontLeftTurnCanId = 0;
        backLeftTurnCanId = 0;
        frontRightTurnCanId = 0;
        backRightTurnCanId = 0;

        frontLeftAbsoluteEncoderCanId = 0;
        backLeftAbsoluteEncoderCanId = 0;
        frontRightAbsoluteEncoderCanId = 0;
        backRightAbsoluteEncoderCanId = 0;

        // Algae Effector (Motor IDs)
        PIVOT_ID = 0;
        REMOVAL_ID = 0;
      }
    }
  }
}
