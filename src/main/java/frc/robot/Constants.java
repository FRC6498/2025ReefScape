package frc.robot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;

public class Constants {
    public class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 24;
        public static final double INTAKE_DEFAULT_SPEED = -0.40; //TODO: find the optimal intake speed
        public static final int CANRANGE_SENSOR_ID = 25;
    }
    public class ArmConstants {
        public static final int ARM_MOTOR_ID = 23;
        public static final Slot0Configs ARM_MOTOR_CONFIG = new Slot0Configs() //TODO: run sysid on all the lift motors
        .withKA(0.0028603)//Feedforward gains
        .withKG(0.05141)
        .withKS(0.20093)
        .withKV(0.1069)
        .withKP(0.010679)//PID 
        .withKI(0)
        .withKD(0);
        public static final double ARM_MAX_VELOCITY = 0.5;
        public static final double ARM_MAX_ACCELERATION = 0.5;
        public static final double Dt = 0.1;
        public static final MotionMagicConfigs ARM_MOTION_CONFIGS = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(60) // target velocity
            .withMotionMagicAcceleration(120) // max acceleration
            .withMotionMagicJerk(1000); //how fast to change acceleration
    }
    public class VisionConstants {
        public static final String LIMELIGHT_NAME = "limelight"; //TODO: set the limelight name
    }
    public static final class LiftConstants {
        public static final int RIGHT_LIFT_MOTOR_ID = 22; //TODO: set the ID's for the lift motors 
        public static final int LEFT_LIFT_MOTOR_ID = 21;
        //find all of these using sysid
        public static final Slot0Configs LIFT_MOTOR_CONFIG = new Slot0Configs() //TODO: run sysid on all the lift motors
        .withKA(0)//Feedforward gains
        .withKG(0)
        .withKS(0)
        .withKV(0)
        .withKP(0)//PID 
        .withKI(0)
        .withKD(0);
        //Trapazoid profile configuration
        //keep it slow at the start so you dont break something
        public static final double MAX_VELOCITY = 0.5;
        public static final double MAX_ACCELERATION = 0.1;
        public static final double Dt = 0.1; // time to advance trapazoid profile calculations(seconds) 
        public static final MotionMagicConfigs LIFT_MOTION_CONFIGS = new MotionMagicConfigs()
            .withMotionMagicAcceleration(5)
            .withMotionMagicCruiseVelocity(5)
            .withMotionMagicJerk(2);      
    }
}
