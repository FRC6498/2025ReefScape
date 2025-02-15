package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

public class Constants {
    public class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 24;
        public static final double INTAKE_DEFAULT_SPEED = 0.5; //TODO: find the optimal intake speed
        public static final int CANRANGE_SENSOR_ID = 25;
    }
    public class VisionConstants {
        public static final String LIMELIGHT_NAME = "limelight"; //TODO: set the limelight name
    }
    public static final class LiftConstants {
        public static final int RIGHT_LIFT_MOTOR_ID = 2; //TODO: set the ID's for the lift motors 
        public static final int LEFT_LIFT_MOTOR_ID = 3;
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
        public static final double Dt = 0.25; // time to advance trapazoid profile calculations(seconds)        
    }
}
