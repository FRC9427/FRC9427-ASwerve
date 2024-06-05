
package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * 請勿向此類添加任何靜態變量，或進行任何初始化。
 * 除非您知道自己在做什麼，否則不要修改此檔案，除了更改 startRobot 調用的參數類。
 */
public final class Main {

    private Main() {
    }

    /**
     * 主初始化功能。請勿在此處進行任何初始化。
     *
     * <p>
     * 如果您更改了主機器人類，請更改參數類型。
     */
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
