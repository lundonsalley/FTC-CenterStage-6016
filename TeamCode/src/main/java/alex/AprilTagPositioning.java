package alex;

public class AprilTagPositioning {
    public static class Pos {
        public static Double x = 0.0;
        public static Double y = 0.0;
        public static Double z = 0.0;
        public static Double pitch = 0.0;
        public static Double roll = 0.0;
        public static Double yaw = 0.0;
        public static Double range = 0.0;
        public static Double bearing = 0.0;
        public static Double elevation = 0.0;
        public static int angle = 0;
        public static double dx = 0.0;
        public static double dy = 0.0;
        public static double dz = 0.0;
        public static double dpitch = 0.0;
        public static double droll = 0.0;
        public static double dyaw = 0.0;
        public static double drange = 0.0;
        public static double dbearing = 0.0;
        public static double delevation = 0.0;
        public static int dangle = 0;
    }
    public static class Target {
        public static Double pos[][] = new Double[3][3];
        public static int angle = 0;
    }
    public static void updateDeltas(){
        Pos.dx = (Target.pos[0][0] != null) ? Pos.x - Target.pos[0][0] : 0;
        Pos.dy = (Target.pos[0][1] != null) ? Pos.y - Target.pos[0][1] : 0;
        Pos.dz = (Target.pos[0][2] != null) ? Pos.z - Target.pos[0][2] : 0;

        Pos.dpitch = (Target.pos[1][0] != null) ? Pos.pitch - Target.pos[1][0] : 0;
        Pos.droll = (Target.pos[1][1] != null) ? Pos.roll - Target.pos[1][1] : 0;
        Pos.dyaw = (Target.pos[1][2] != null) ? Target.pos[1][2] - Pos.yaw : 0;

        Pos.drange = (Target.pos[2][0] != null) ? Pos.range - Target.pos[2][0] : 0;
        Pos.dbearing = (Target.pos[2][1] != null) ? Pos.bearing - Target.pos[2][1] : 0;
        Pos.delevation = (Target.pos[2][2] != null) ? Pos.elevation - Target.pos[2][2] : 0;

        Pos.dangle = Target.angle - Pos.angle;
    }

}
