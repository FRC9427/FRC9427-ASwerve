
package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.io.IOException;
import swervelib.parser.SwerveParser;

public class Robot extends TimedRobot {

    private static Robot instance;
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private Timer disabledTimer;

    public Robot() {
        instance = this;
    }

    public static Robot getInstance() {
        return instance;
    }

    /**
     * 當機器人第一次啟動時運行此功能，應用於任何初始化代碼。
     */
    @Override
    public void robotInit() {
        // 實例化我們的 RobotContainer。這將執行我們所有的按鈕綁定，
        // 並將我們的自動選擇器放在儀表板上。
        m_robotContainer = new RobotContainer();

        // 創建一個計時器，在禁用後幾秒鐘禁用電機煞車。這將讓機器人在禁用時立即停止，
        // 但也允許它被推動。
        disabledTimer = new Timer();
    }

    /**
     * 每 20 毫秒調用一次此功能，無論模式如何。用於您希望在禁用、自主、遠程操作和測試期間運行的項目，如診斷。
     *
     * <p>
     * 這在特定模式的週期性功能之後運行，但在 LiveWindow 和 SmartDashboard 集成更新之前。
     */
    @Override
    public void robotPeriodic() {
        // 運行調度器。這負責輪詢按鈕、新添加的計劃命令、運行已經計劃的命令、刪除完成或中斷的命令，
        // 並運行子系統的 periodic() 方法。這必須從機器人的 periodic 區塊中調用，以便基於命令的框架中的任何內容正常工作。
        CommandScheduler.getInstance().run();
    }

    /**
     * 每次機器人進入禁用模式時調用此功能。
     */
    @Override
    public void disabledInit() {
        m_robotContainer.setMotorBrake(true);
        disabledTimer.reset();
        disabledTimer.start();
    }

    @Override
    public void disabledPeriodic() {
        if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
            m_robotContainer.setMotorBrake(false);
            disabledTimer.stop();
        }
    }

    /**
     * 此自主命令運行您在 {@link RobotContainer} 類中選擇的自主命令。
     */
    @Override
    public void autonomousInit() {
        m_robotContainer.setMotorBrake(true);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // 計劃自動命令（範例）
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * 自主期間定期調用此功能。
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // 這確保當遠程操作開始運行時，自主停止運行。如果您希望自主繼續，直到被另一個命令中斷，請刪除此行或將其註釋掉。
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        // m_robotContainer.setDriveMode();
        m_robotContainer.setMotorBrake(true);
    }

    /**
     * 遠程操作控制期間定期調用此功能。
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // 在測試模式開始時取消所有運行的命令。
        CommandScheduler.getInstance().cancelAll();
        try {
            new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * 測試模式期間定期調用此功能。
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * 機器人第一次啟動時調用此功能。
     */
    @Override
    public void simulationInit() {
    }

    /**
     * 模擬期間定期調用此功能。
     */
    @Override
    public void simulationPeriodic() {
    }
}
