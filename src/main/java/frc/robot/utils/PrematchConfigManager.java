package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

public class PrematchConfigManager {
    private static PrematchConfigManager instance;

    // Keys
    private static final String KEY_INPUT = "DASHBOARD/PrematchConfig/InputJSON";
    private static final String KEY_STATUS = "DASHBOARD/PrematchConfig/Status";
    private static final String KEY_VERIFICATION_CODE = "DASHBOARD/PrematchConfig/VerificationCode";

    private final NetworkTableEntry inputEntry;
    private final NetworkTableEntry statusEntry;
    private final NetworkTableEntry verificationCodeEntry;

    private final Field2d prematchField = new Field2d();

    private boolean parsedSuccessfully = false;

    // Parsed poses (Blue origin)
    private Pose3d leftShortPassTarget;
    private Pose3d rightShortPassTarget;
    private Pose3d leftLongPassTarget;
    private Pose3d rightLongPassTarget;
    private Pose2d leftPathfindTarget;
    private Pose2d rightPathfindTarget;
    private Pose3d autoTarget;

    private final ObjectMapper mapper = new ObjectMapper();
    private String lastJson = "";

    private PrematchConfigManager() {
        var table = NetworkTableInstance.getDefault().getTable("");
        inputEntry = table.getEntry(KEY_INPUT);
        statusEntry = table.getEntry(KEY_STATUS);
        verificationCodeEntry = table.getEntry(KEY_VERIFICATION_CODE);

        // Ensure keys exist
        inputEntry.setDefaultString("");
        statusEntry.setDefaultString("Initialized");
        verificationCodeEntry.setDefaultString("");

        SmartDashboard.putData("DASHBOARD/PrematchConfig/Field", prematchField);
    }

    public static PrematchConfigManager getInstance() {
        if (instance == null) {
            instance = new PrematchConfigManager();
        }
        return instance;
    }

    public void periodic() {
        String currentJson = inputEntry.getString("");
        if (!currentJson.equals(lastJson) && !currentJson.isEmpty()) {
            parse(currentJson);
            lastJson = currentJson;
        } else if (currentJson.isEmpty() && !lastJson.isEmpty()) {
            // Cleared out
            parsedSuccessfully = false;
            statusEntry.setString("fallback-defaults");
            verificationCodeEntry.setString("");
            lastJson = "";
        }
    }

    private void parse(String jsonString) {
        try {
            JsonNode root = mapper.readTree(jsonString);

            // Validate format and version
            if (!root.has("version") || root.get("version").asInt() != 2) throw new Exception("Invalid version");
            if (!root.has("format") || !root.get("format").asText().equals("FRC26v2")) throw new Exception("Invalid format");
            
            String code = root.has("verificationCode") ? root.get("verificationCode").asText() : "N/A";

            // Parse arrays
            JsonNode cpArr = root.get("closePassPoses");
            JsonNode lpArr = root.get("longPassPoses");
            JsonNode pfArr = root.get("pathfindPoses");
            JsonNode atNode = root.get("autoTarget");

            if (cpArr == null || cpArr.size() != 2) throw new Exception("Invalid closePassPoses");
            if (lpArr == null || lpArr.size() != 2) throw new Exception("Invalid longPassPoses");
            if (pfArr == null || pfArr.size() != 2) throw new Exception("Invalid pathfindPoses");
            if (atNode == null) throw new Exception("Invalid autoTarget");

            leftShortPassTarget = parsePose3d(getNodeById(cpArr, "closePassLeft"));
            rightShortPassTarget = parsePose3d(getNodeById(cpArr, "closePassRight"));
            
            leftLongPassTarget = parsePose3d(getNodeById(lpArr, "longPassLeft"));
            rightLongPassTarget = parsePose3d(getNodeById(lpArr, "longPassRight"));

            leftPathfindTarget = parsePose2d(getNodeById(pfArr, "PF1"));
            rightPathfindTarget = parsePose2d(getNodeById(pfArr, "PF2"));

            autoTarget = parsePose3d(atNode);

            parsedSuccessfully = true;
            statusEntry.setString("parsed");
            verificationCodeEntry.setString(code);

            // Update Field2d
            prematchField.getObject("ClosePass").setPoses(leftShortPassTarget.toPose2d(), rightShortPassTarget.toPose2d());
            prematchField.getObject("LongPass").setPoses(leftLongPassTarget.toPose2d(), rightLongPassTarget.toPose2d());
            prematchField.getObject("Pathfind").setPoses(leftPathfindTarget, rightPathfindTarget);
            prematchField.getObject("AutoTarget").setPose(autoTarget.toPose2d());

        } catch (Exception e) {
            System.err.println("Failed to parse prematch config: " + e.getMessage());
            parsedSuccessfully = false;
            statusEntry.setString("fallback-defaults");
            verificationCodeEntry.setString("");
            
            // clear field
            prematchField.getObject("ClosePass").setPoses();
            prematchField.getObject("LongPass").setPoses();
            prematchField.getObject("Pathfind").setPoses();
            prematchField.getObject("AutoTarget").setPoses();
        }
    }

    private Pose3d parsePose3d(JsonNode node) throws Exception {
        double x = getValidatedDouble(node, "xMeters");
        double y = getValidatedDouble(node, "yMeters");
        return new Pose3d(x, y, 0, new Rotation3d());
    }

    private Pose2d parsePose2d(JsonNode node) throws Exception {
        double x = getValidatedDouble(node, "xMeters");
        double y = getValidatedDouble(node, "yMeters");
        // default 0 rotation for parsed data as rotation wasn't specified in JSON sample
        return new Pose2d(x, y, new Rotation2d());
    }

    private JsonNode getNodeById(JsonNode array, String id) throws Exception {
        for (JsonNode node : array) {
            if (node.has("id") && node.get("id").asText().equals(id)) {
                return node;
            }
        }
        throw new Exception("Node with id " + id + " not found");
    }

    private double getValidatedDouble(JsonNode node, String key) throws Exception {
        if (!node.has(key)) throw new Exception("Missing key: " + key);
        double val = node.get(key).asDouble();
        if (!Double.isFinite(val)) throw new Exception("Non-finite value for: " + key);
        return val;
    }

    public boolean isParsedSuccessfully() { return parsedSuccessfully; }

    public Pose3d getLeftShortPassTarget() { return leftShortPassTarget; }
    public Pose3d getRightShortPassTarget() { return rightShortPassTarget; }
    public Pose3d getLeftLongPassTarget() { return leftLongPassTarget; }
    public Pose3d getRightLongPassTarget() { return rightLongPassTarget; }
    public Pose2d getLeftPathfindTarget() { return leftPathfindTarget; }
    public Pose2d getRightPathfindTarget() { return rightPathfindTarget; }
    public Pose3d getAutoTarget() { return autoTarget; }
}       
