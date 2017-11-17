#include <Eigen/Core>
#include <vector>
#include <string>

// PlyPoint is a basic structure for a 3D point
// It is a location and a colour, represented by Eigen vectors.
struct PlyPoint {
  Eigen::Vector3d location; // Location of the point, as a vector of three doubles
  Eigen::Vector3i colour; // Colour of the point, as a vector of three integers (typically values in 0-255)
};

// SimplePly is a basic storage class for a point cloud.
// It is essentially a thin wrapper around a std::vector of PlyPoints.
class SimplePly {
 public:

  // Read points from a PLY file (faces, etc. are ignored)
  bool read(const std::string& filename);
    
  // Write points to a PLY file (in ASCII format)
  bool write(const std::string& filename);

  // Get the number of points in the cloud
  size_t size() { return points_.size(); }
    
  // Array-style access for points in the cloud.
  // Since a reference is returned, this allows modification of the points.
  PlyPoint& operator[](size_t ix) { return points_[ix]; }
    
  // const access for points in the cloud.
  // Since a const reference is returned, no modification is allowed.
  const PlyPoint& operator[](size_t ix) const { return points_[ix]; }

 private:

  std::vector<PlyPoint> points_; // Internal storage of the point cloud.

};
