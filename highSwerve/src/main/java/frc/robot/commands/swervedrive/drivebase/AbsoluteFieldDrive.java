
package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * 一個使用示例子系統的示例命令。
 */
public class AbsoluteFieldDrive extends Command {

    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY, heading;

    /**
     * 用於在全場中心模式下驅動Swerve機器人。vX和vY提供平移輸入，其中x是
     * 向/遠離聯盟牆壁，y是左/右。headingHorzontal和headingVertical是機器人角度的笛卡爾坐標，
     * 它們將被轉換為極角，機器人將旋轉到該角度。
     *
     * @param swerve  Swerve驅動底盤子系統。
     * @param vX      提供x-平移搖桿輸入的DoubleSupplier。
     *                應該在-1到1的範圍內，已經考慮了死區。正的X是遠離聯盟牆壁。
     * @param vY      提供y-平移搖桿輸入的DoubleSupplier。
     *                應該在-1到1的範圍內，已經考慮了死區。正的Y是當透過駕駛員站玻璃觀看時，朝向左牆。
     * @param heading 提供機器人的頭部角度的DoubleSupplier。
     */
    public AbsoluteFieldDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
            DoubleSupplier heading) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.heading = heading;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    // 在計劃的命令被調度時，每次調度器運行時調用。
    @Override
    public void execute() {

        // 根據2個搖桿模塊獲取期望的底盤速度。

        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
                new Rotation2d(heading.getAsDouble() * Math.PI));

        // 限制速度以防止翻轉
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                swerve.getSwerveDriveConfiguration());
        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());

        // 讓機器人移動
        swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);

    }

    // 當命令結束或被中斷時調用一次。
    @Override
    public void end(boolean interrupted) {
    }

    // 當命令應該結束時返回true。
    @Override
    public boolean isFinished() {
        return false;
    }

}
