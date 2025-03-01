package frc.robot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.config.RobotConfig;

public class Constants {
    public class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 24;
        public static final double INTAKE_DEFAULT_SPEED = -0.40; 
        public static final int CANRANGE_SENSOR_ID = 25;
    }
    public class ArmConstants {
        public static final int ARM_MOTOR_ID = 23;
        public static final Slot0Configs ARM_MOTOR_CONFIG = new Slot0Configs() 
        .withKA(0.0028603)//Feedforward gains
        .withKG(-0.20141)
        .withKS(0.20093)
        .withKV(0.1069)
        .withKP(0.010679)//PID 
        .withKI(0)
        .withKD(0);
        public static final MotionMagicConfigs ARM_MOTION_CONFIGS = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(60) // max velocity
            .withMotionMagicAcceleration(120) // max acceleration
            .withMotionMagicJerk(1000); //max change in acceleration
    }
    public class VisionConstants {
        public static final String LIMELIGHT_NAME = "limelight"; //TODO: set the limelight name
    }
    public static final class LiftConstants {
        public static final int RIGHT_LIFT_MOTOR_ID = 22; 
        public static final int LEFT_LIFT_MOTOR_ID = 21;
        //find all of these using sysid
        public static final Slot0Configs LIFT_MOTOR_CONFIG = new Slot0Configs() //TODO: run sysid on all the lift motors
        .withKA(.0098891)//Feedforward gains
        .withKG(.47469)
        .withKS(.036355)
        .withKV(.12215)
        .withKP(.1358)//PID 
        .withKI(0)
        .withKD(.1);
        public static final MotionMagicConfigs LIFT_MOTION_CONFIGS = new MotionMagicConfigs()
            .withMotionMagicAcceleration(100)
            .withMotionMagicCruiseVelocity(100)
            .withMotionMagicJerk(500);      
    }
    public static final class RobotConstants{
        public static RobotConfig config;

    }
}
