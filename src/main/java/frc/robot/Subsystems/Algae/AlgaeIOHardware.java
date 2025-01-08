package frc.robot.Subsystems.Algae;

import edu.wpi.first.math.util.Units;

public class AlgaeIOHardware implements AlgaeIO {
    private static final double WRIST_GEAR_RATIO = 1.0;
    private static final double WRIST_POSITION_COEFFICIENT = (2 * Math.PI) *  (WRIST_GEAR_RATIO * 2048); // idk why we muliply by 2048 but we do
    private static final double WRIST_VELOCITY_COEFFICIENT = WRIST_POSITION_COEFFICIENT * 10;

    public static final double WRIST_SLOW_ACCELERATION = Units.degreesToRadians(500);
    public static final double WRIST_FAST_ACCELERATION = Units.degreesToRadians(750);
    public static final double WRIST_VELOCITY = Units.degreesToRadians(300);

    private static final double WRIST_SLOW_ACCELERATION_CONSTRAINT =
    WRIST_SLOW_ACCELERATION / WRIST_VELOCITY_COEFFICIENT;
    private static final double SHOULDER_FAST_ACCELERATION_CONSTRAINT =
    WRIST_FAST_ACCELERATION / WRIST_VELOCITY_COEFFICIENT;
    private static final double SHOULDER_VELOCITY_CONSTRAINT = 
    WRIST_VELOCITY / WRIST_VELOCITY_COEFFICIENT;   
}
