package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;
import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.auto.PIDConstants;
import frc.robot.Constants.Auton;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import frc.robot.Constants;
import swervelib.parser.PIDFConfig;


public class Auto  {
    public static CommandBase auto(SwerveSubsystem swerve){
      PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new PathConstraints(8, 8));
      PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);
      System.out.println(exampleState.velocityMetersPerSecond);

      //List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Example Path Group", new PathConstraints(4, 3));

      /*List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup(
        "Example Path Group", 
        new PathConstraints(4, 3), 
        new PathConstraints(2, 2), 
        new PathConstraints(3, 3));*/

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));
       
        /*HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
          swerve::getPose, // Pose2d supplier
          swerve::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
          swerve.kinematics, // SwerveDriveKinematics
          new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
          new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
          swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
          eventMap,
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          swerve // The drive subsystem. Used to properly set the requirements of path following commands
        );*/

        


        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
          swerve::getPose,
// Pose2d supplier
          swerve::resetOdometry,
// Pose2d consumer, used to reset odometry at the beginning of auto
          new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d),
// PID constants to correct for translation error (used to create the X and Y PID controllers)
          new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d),
// PID constants to correct for rotation error (used to create the rotation controller)
          swerve::setChassisSpeeds,
// Module states consumer used to output to the drive subsystem
          eventMap,
          false,
// Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          swerve
// The drive subsystem. Used to properly set the requirements of path following commands
        ); 

        Command fullAuto = autoBuilder.fullAuto(pathGroup);


      return Commands.sequence(autoBuilder.fullAuto(examplePath));




      
    }
    
}