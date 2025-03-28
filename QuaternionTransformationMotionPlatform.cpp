/**
 * Platform Actuator Length Calculator
 *
 * This program calculates the required actuator lengths for a 4-corner platform
 * based on heave, pitch, and roll transformations.
 *
 * Platform dimensions: 42" x 72" (width x length)
 * Actuator stroke: 2 inches
 * Neutral height: 1 inch
 */

#include <iostream>
#include <cmath>
#include <vector>
#include <string>

 // Define M_PI if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Define the coordinate structure
struct Point3D {
    double x, y, z;

    Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    // Vector subtraction
    Point3D operator-(const Point3D& p) const {
        return Point3D(x - p.x, y - p.y, z - p.z);
    }

    // Vector addition
    Point3D operator+(const Point3D& p) const {
        return Point3D(x + p.x, y + p.y, z + p.z);
    }

    // Magnitude of the vector
    double magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }
};

// Define the quaternion structure for rotations
struct Quaternion {
    double w, x, y, z;

    Quaternion(double w = 1, double x = 0, double y = 0, double z = 0) : w(w), x(x), y(y), z(z) {}

    // Create a quaternion from axis-angle representation
    static Quaternion fromAxisAngle(const Point3D& axis, double angle) {
        // Calculate the magnitude of the axis vector
        double magnitude = axis.magnitude();

        // Normalize the axis
        Point3D normalizedAxis(axis.x / magnitude, axis.y / magnitude, axis.z / magnitude);

        // Calculate half-angle values
        double halfAngle = angle / 2.0;
        double sinHalfAngle = std::sin(halfAngle);

        return Quaternion(
            std::cos(halfAngle),
            normalizedAxis.x * sinHalfAngle,
            normalizedAxis.y * sinHalfAngle,
            normalizedAxis.z * sinHalfAngle
        );
    }

    // Multiply two quaternions (compose rotations)
    Quaternion multiply(const Quaternion& q) const {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }

    // Calculate the inverse of the quaternion
    Quaternion inverse() const {
        double norm = w * w + x * x + y * y + z * z;
        return Quaternion(w / norm, -x / norm, -y / norm, -z / norm);
    }

    // Rotate a vector using this quaternion
    Point3D rotateVector(const Point3D& v) const {
        // Create a quaternion from the vector
        Quaternion vectorQ(0, v.x, v.y, v.z);

        // Apply rotation: q * v * q^-1
        Quaternion result = this->multiply(vectorQ).multiply(this->inverse());

        return Point3D(result.x, result.y, result.z);
    }

    // Normalize the quaternion
    Quaternion normalize() const {
        double magnitude = std::sqrt(w * w + x * x + y * y + z * z);
        return Quaternion(w / magnitude, x / magnitude, y / magnitude, z / magnitude);
    }
};

// Define the Platform class
class Platform {
private:
    // Base connection points (on ground)
    Point3D basePoints[4];

    // Platform connection points (on platform)
    Point3D platformPoints[4];

    // Current platform connection points after transformations
    Point3D currentPlatformPoints[4];

    // Current height (heave)
    double currentHeight;

    // Current orientation
    double currentPitch;
    double currentRoll;

    // Platform dimensions
    double width;
    double length;

    // Actuator limits
    double minActuatorLength;
    double maxActuatorLength;

    // Max pitch and roll limits in radians
    double maxPitch;
    double maxRoll;

public:
    // Constructor
    Platform(double width, double length, double neutralHeight,
        double minActuatorLength, double maxActuatorLength,
        double maxPitchDegrees, double maxRollDegrees) {

        this->width = width;
        this->length = length;
        this->currentHeight = neutralHeight;
        this->currentPitch = 0;
        this->currentRoll = 0;
        this->minActuatorLength = minActuatorLength;
        this->maxActuatorLength = maxActuatorLength;

        // Convert max pitch and roll to radians
        this->maxPitch = maxPitchDegrees * M_PI / 180.0;
        this->maxRoll = maxRollDegrees * M_PI / 180.0;

        // Initialize base points (ground level)
        // Order: Rear Left, Rear Right, Front Left, Front Right
        basePoints[0] = Point3D(0, 0, 0);          // Rear Left
        basePoints[1] = Point3D(width, 0, 0);      // Rear Right
        basePoints[2] = Point3D(0, length, 0);     // Front Left
        basePoints[3] = Point3D(width, length, 0); // Front Right

        // Initialize platform points at neutral position
        // 1 inch above the ground (z = neutralHeight)
        platformPoints[0] = Point3D(0, 0, neutralHeight);          // Rear Left
        platformPoints[1] = Point3D(width, 0, neutralHeight);      // Rear Right
        platformPoints[2] = Point3D(0, length, neutralHeight);     // Front Left
        platformPoints[3] = Point3D(width, length, neutralHeight); // Front Right

        // Initialize current platform points
        for (int i = 0; i < 4; i++) {
            currentPlatformPoints[i] = platformPoints[i];
        }

        std::cout << "Platform initialized with dimensions: " << width << "\" x " << length << "\"" << std::endl;
        std::cout << "Platform center at: (" << width / 2 << ", " << length / 2 << ")" << std::endl;
        std::cout << "Neutral height: " << neutralHeight << "\"" << std::endl;
        std::cout << "Max pitch: " << maxPitchDegrees << "°, Max roll: " << maxRollDegrees << "°" << std::endl;
        std::cout << "Actuator limits: " << minActuatorLength << "\" to " << maxActuatorLength << "\"" << std::endl;
        std::cout << std::endl;
    }

    // Apply transformations to the platform
    bool applyTransformation(double heave, double pitchDegrees, double rollDegrees) {
        // Convert pitch and roll to radians
        // Negate pitch to reverse the orientation (positive pitch = front down, rear up)
        double pitch = -pitchDegrees * M_PI / 180.0;
        double roll = rollDegrees * M_PI / 180.0;

        // Update current state
        double newHeight = currentHeight + heave;
        double newPitch = currentPitch + pitch;
        double newRoll = currentRoll + roll;

        // Check if the new orientation is within limits
        if (std::abs(newPitch) > maxPitch || std::abs(newRoll) > maxRoll) {
            std::cout << "Error: Requested orientation exceeds platform limits." << std::endl;
            return false;
        }

        // Create rotation quaternions
        Point3D pitchAxis(1, 0, 0);  // Pitch around X-axis
        Point3D rollAxis(0, 1, 0);   // Roll around Y-axis

        Quaternion qPitch = Quaternion::fromAxisAngle(pitchAxis, pitch);
        Quaternion qRoll = Quaternion::fromAxisAngle(rollAxis, roll);

        // Combined rotation (roll after pitch)
        Quaternion qRotation = qRoll.multiply(qPitch);

        // Calculate platform center
        Point3D center(width / 2, length / 2, currentHeight);

        // Apply transformations to each platform point
        for (int i = 0; i < 4; i++) {
            // Start with neutral position
            Point3D point = platformPoints[i];

            // Translate point to origin (relative to center)
            point.x -= center.x;
            point.y -= center.y;
            point.z -= center.z + platformPoints[i].z - currentHeight;

            // Apply existing rotations (current state)
            if (currentPitch != 0 || currentRoll != 0) {
                Quaternion qCurrentPitch = Quaternion::fromAxisAngle(pitchAxis, currentPitch);
                Quaternion qCurrentRoll = Quaternion::fromAxisAngle(rollAxis, currentRoll);
                Quaternion qCurrent = qCurrentRoll.multiply(qCurrentPitch);

                // Apply current rotation around center
                point = qCurrent.rotateVector(point);
            }

            // Apply new rotation around center
            point = qRotation.rotateVector(point);

            // Translate back to world coordinates and apply heave
            point.x += center.x;
            point.y += center.y;
            point.z += center.z + heave;

            // Store the new platform point
            currentPlatformPoints[i] = point;
        }

        // Update current state
        currentHeight = newHeight;
        currentPitch = newPitch;
        currentRoll = newRoll;

        // Check if actuator lengths are within limits
        if (!validateActuatorLengths()) {
            // Revert to previous state if invalid
            currentHeight -= heave;
            currentPitch -= pitch;
            currentRoll -= roll;

            // Recalculate platform points
            // Calculate platform center
            Point3D center(width / 2, length / 2, currentHeight);

            for (int i = 0; i < 4; i++) {
                // Start with neutral position
                Point3D point = platformPoints[i];

                // Translate point to origin (relative to center)
                point.x -= center.x;
                point.y -= center.y;
                point.z -= center.z + platformPoints[i].z - currentHeight;

                // Apply existing rotations (current state)
                if (currentPitch != 0 || currentRoll != 0) {
                    Quaternion qCurrentPitch = Quaternion::fromAxisAngle(pitchAxis, currentPitch);
                    Quaternion qCurrentRoll = Quaternion::fromAxisAngle(rollAxis, currentRoll);
                    Quaternion qCurrent = qCurrentRoll.multiply(qCurrentPitch);

                    // Apply current rotation around center
                    point = qCurrent.rotateVector(point);
                }

                // Translate back to world coordinates
                point.x += center.x;
                point.y += center.y;
                point.z += center.z;

                // Store the new platform point
                currentPlatformPoints[i] = point;
            }

            std::cout << "Error: Requested transformation would exceed actuator limits." << std::endl;
            return false;
        }

        return true;
    }

    // Calculate and validate actuator lengths
    bool validateActuatorLengths() {
        double neutralLength = platformPoints[0].z; // Initial height is the neutral length
        double maxStroke = maxActuatorLength - neutralLength;

        for (int i = 0; i < 4; i++) {
            Point3D actuatorVector = currentPlatformPoints[i] - basePoints[i];
            double length = actuatorVector.magnitude();

            // Calculate how far the actuator has moved from neutral
            double strokeUsed = std::abs(length - neutralLength);

            // Check if we're exceeding stroke limits
            if (strokeUsed > maxStroke) {
                std::cout << "Actuator " << i << " exceeds stroke limit. Stroke used: "
                    << strokeUsed << ", Max stroke: " << maxStroke << std::endl;
                return false;
            }
        }
        return true;
    }

    // Get the current actuator lengths
    std::vector<double> getActuatorLengths() const {
        std::vector<double> lengths(4);

        for (int i = 0; i < 4; i++) {
            Point3D actuatorVector = currentPlatformPoints[i] - basePoints[i];
            lengths[i] = actuatorVector.magnitude();
        }

        return lengths;
    }

    // Get actuator stroke usage
    std::vector<double> getActuatorStrokes() const {
        std::vector<double> strokes(4);
        double neutralLength = platformPoints[0].z; // Initial height is the neutral length

        for (int i = 0; i < 4; i++) {
            Point3D actuatorVector = currentPlatformPoints[i] - basePoints[i];
            double length = actuatorVector.magnitude();
            strokes[i] = length - neutralLength;
        }

        return strokes;
    }

    // Get actuator angles (from vertical)
    std::vector<std::pair<double, double>> getActuatorAngles() const {
        std::vector<std::pair<double, double>> angles(4);

        for (int i = 0; i < 4; i++) {
            Point3D actuatorVector = currentPlatformPoints[i] - basePoints[i];

            // Calculate angle in X-direction (roll angle from vertical)
            double angleX = std::atan2(actuatorVector.x, actuatorVector.z) * 180.0 / M_PI;

            // Calculate angle in Y-direction (pitch angle from vertical)
            double angleY = std::atan2(actuatorVector.y, actuatorVector.z) * 180.0 / M_PI;

            angles[i] = std::make_pair(angleX, angleY);
        }

        return angles;
    }

    // Get current platform state
    void getCurrentState(double& height, double& pitch, double& roll) const {
        height = currentHeight;
        // Negate pitch to match the input convention (positive pitch = front down)
        pitch = -currentPitch * 180.0 / M_PI;  // Convert to degrees
        roll = currentRoll * 180.0 / M_PI;    // Convert to degrees
    }

    // Reset platform to neutral position
    void reset() {
        currentHeight = platformPoints[0].z;  // Initial height from constructor
        currentPitch = 0;
        currentRoll = 0;

        for (int i = 0; i < 4; i++) {
            currentPlatformPoints[i] = platformPoints[i];
        }
    }
};

// Print the actuator lengths and angles
void printActuatorInfo(const Platform& platform) {
    // Get actuator lengths
    std::vector<double> lengths = platform.getActuatorLengths();

    // Get actuator strokes
    std::vector<double> strokes = platform.getActuatorStrokes();

    // Get actuator angles
    std::vector<std::pair<double, double>> angles = platform.getActuatorAngles();

    // Get current platform state
    double height, pitch, roll;
    platform.getCurrentState(height, pitch, roll);

    // Print platform state
    std::cout << "Platform State:" << std::endl;
    std::cout << "  Height: " << height << " inches" << std::endl;
    std::cout << "  Pitch:  " << pitch << " degrees" << std::endl;
    std::cout << "  Roll:   " << roll << " degrees" << std::endl;

    // Print actuator information
    std::cout << "Actuator Lengths:" << std::endl;
    std::cout << "  Rear Left (0):  " << lengths[0] << " inches (stroke: " << strokes[0] << " inches)" << std::endl;
    std::cout << "  Rear Right (1): " << lengths[1] << " inches (stroke: " << strokes[1] << " inches)" << std::endl;
    std::cout << "  Front Left (2): " << lengths[2] << " inches (stroke: " << strokes[2] << " inches)" << std::endl;
    std::cout << "  Front Right (3): " << lengths[3] << " inches (stroke: " << strokes[3] << " inches)" << std::endl;

    std::cout << "Actuator Angles (from vertical):" << std::endl;
    for (int i = 0; i < 4; i++) {
        std::cout << "  Actuator " << i << ": Roll angle = " << angles[i].first
            << "°, Pitch angle = " << angles[i].second << "°" << std::endl;
    }

    std::cout << std::endl;
}

int main() {
    // Platform configuration
    double width = 42.0;         // Platform width in inches
    double length = 72.0;        // Platform length in inches
    double neutralHeight = 1.0;  // Neutral height in inches

    // Actuator limits
    double minActuatorLength = 0.0;   // Minimum actuator length
    double maxActuatorLength = 3.0;   // Maximum actuator length (1 inch + 2 inch stroke)

    // Maximum angles
    double maxPitchDegrees = 3.18;    // Maximum pitch in degrees
    double maxRollDegrees = 5.47;     // Maximum roll in degrees

    // Create the platform
    Platform platform(width, length, neutralHeight, minActuatorLength, maxActuatorLength,
        maxPitchDegrees, maxRollDegrees);

    // Print initial state
    std::cout << "Initial Platform State:" << std::endl;
    printActuatorInfo(platform);

    // Interactive mode
    char choice;
    do {
        double heave, pitch, roll;

        std::cout << "Enter transformation:" << std::endl;
        std::cout << "Heave (inches): ";
        std::cin >> heave;
        std::cout << "Pitch (degrees): ";
        std::cin >> pitch;
        std::cout << "Roll (degrees): ";
        std::cin >> roll;

        // Apply the transformation
        if (platform.applyTransformation(heave, pitch, roll)) {
            std::cout << "Transformation applied successfully." << std::endl;
            printActuatorInfo(platform);
        }
        else {
            std::cout << "Failed to apply transformation." << std::endl;
        }

        std::cout << "Continue? (y/n): ";
        std::cin >> choice;
    } while (choice == 'y' || choice == 'Y');

    return 0;
}