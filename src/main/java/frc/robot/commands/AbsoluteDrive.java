package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;
import frc.robot.subsystems.SwerveSubsystem;

public class AbsoluteDrive extends CommandBase {

    private final SwerveSubsystem swerve;
    private final DoubleSupplier  vX, vY;
    private final DoubleSupplier headingHorizontal, headingVertical;
    private final boolean isOpenLoop;

    public AbsoluteDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingHorizontal, DoubleSupplier headingVertical, boolean isOpenLoop){
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingHorizontal = headingHorizontal;
        this.headingVertical = headingVertical;
        this.isOpenLoop = isOpenLoop;

        addRequirements(swerve);
    }

    @Override
    public void execute(){

        // Get the desired chassis speeds based on a 2 joystick module.

        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble()/2, vY.getAsDouble()/2,
                                                            headingHorizontal.getAsDouble(),
                                                            headingVertical.getAsDouble());

        // Limit velocity to prevent tippy
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                            Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                                            swerve.getSwerveDriveConfiguration());
        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());

        // Make the robot move
        swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true, isOpenLoop);
    }

    @Override
  public boolean isFinished(){
    return false;
  }
        
}