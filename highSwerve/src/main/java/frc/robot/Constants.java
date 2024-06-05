
package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {

    public static final double ROBOT_MASS = 35; // 公斤
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // 秒，20毫秒 + 110毫秒sprk最大速度滯後

    public static final class AutonConstants {

        public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(5.0, 0, 0.0);
    }

    public static final class DrivebaseConstants {

        // 在禁用時電機制動的保持時間
        public static final double WHEEL_LOCK_TIME = 10; // 秒
    }

    public static class OperatorConstants {

        // 搖桿死區
        public static final double LEFT_X_DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }
}
