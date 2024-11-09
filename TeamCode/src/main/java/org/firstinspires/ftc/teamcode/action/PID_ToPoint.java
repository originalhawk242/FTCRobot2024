package org.firstinspires.ftc.teamcode.action;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.modules.DriveTrain;

@Config
public class PID_ToPoint {
    DriveTrain driveTrain;

    //PIDs for control
    PIDController xController;
    PIDController yController;
    PIDController hController;

    //PID Coefficients for translation
    public static double TRANSLATE_P = 0.5;
    public static double TRANSLATE_I = 0;
    public static double TRANSLATE_D = 0.05;

    //PID Coefficients for rotation
    public static double ROTATE_P = 0.5;
    public static double ROTATE_I = 0;
    public static double ROTATE_D = 0.05;

    public PID_ToPoint(DriveTrain inputdriveTrain){
        // assign object to driveTrain
        driveTrain = inputdriveTrain;

        // initialize PID controllers
        xController = new PIDController(TRANSLATE_P, TRANSLATE_I,TRANSLATE_D);
        yController = new PIDController(TRANSLATE_P, TRANSLATE_I,TRANSLATE_D);
        hController = new PIDController(ROTATE_P, ROTATE_I, ROTATE_D);
    }
}
