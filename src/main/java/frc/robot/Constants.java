
package frc.robot;
public final class Constants {
    /**
    
    * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.56; 
    
    
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.68; 

    public static final int DRIVETRAIN_PIGEON_ID = 0; 

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 11; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 10; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(120.49+180); 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 13; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2;  
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(38.14);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 17;     
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 16;     
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 4; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(226.84+180);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 15; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 14; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(13.53); 
}
