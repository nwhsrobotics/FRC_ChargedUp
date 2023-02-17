package frc.robot;

public final class Constants {

    public static final class ShoulderConstants {

        public static final double kp = 1;
        public static final double ki = 0.0;
        public static final double kd = 0.0;
        public static final double kIz = 0.0;
        public static final double kFFz = 0.0;
        public static final double kMaxOutput = 1.0;
        public static final double kMinOutput = -1.0;
        public static final int ShoulderCanID20 = 20;
        public static final int ShoulderCanID21 = 21;
    }

    public static final class ExtendArmConstants {

        public static final double kp = 1;
        public static final double ki = 0.0;
        public static final double kd = 0.0;
        public static final double kIz = 0.0;
        public static final double kFFz = 0.0;
        public static final double kMaxOutput = 1.0;
        public static final double kMinOutput = -1.0;
        public static final int ExtendArmCanID24 = 3;
        public static final int ExtendArmCanID25 = 25;
    }

    public static final class GrabberConstants {

        
        public static final int forwardChannel = 30;
        public static final int reverseChannel = 40;
    }
}
