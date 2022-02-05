// package frc.robot.Vision;

// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;


// import edu.wpi.first.wpilibj.TimedRobot;

// public class SnakeEyesFetch extends TimedRobot{
//     private static PhotonCamera camera = new PhotonCamera("Team811ObjectCamera");
//     private static PhotonPipelineResult result = camera.getLatestResult();
//     private static PhotonTrackedTarget target = result.getBestTarget();
    
//     private static double x;
//     private static double y;
//     private static double area;
//     private static boolean inVision = result.hasTargets();

//     public static double getX() {
//         if (inVision == true)
//             x = target.getYaw();
//         return x;
//     }

//     public static double getY() {
//         if (inVision == true)
//             y = target.getPitch();
//         return y;
//     }

//     public static double getA() {
//         if (inVision == true)
//             area = target.getArea();
//         return area;
//     }

//     public static boolean getV() {
//         if (inVision == true)
//             inVision = result.hasTargets();
//         return inVision;
//     }
// }
