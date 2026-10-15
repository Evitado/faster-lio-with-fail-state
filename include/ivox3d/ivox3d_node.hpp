#ifndef FASTER_LIO_IVOX3D_NODE_HPP
#define FASTER_LIO_IVOX3D_NODE_HPP

#include <Eigen/Core>
#include <vector>

namespace faster_lio {

// point (anything with x, y, z) to eigen
template <typename T, int dim, typename PointType>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType& pt) {
    return Eigen::Matrix<T, dim, 1>(pt.x, pt.y, pt.z);
}

/// one voxel of the ivox map: a plain list of its points
template <typename PointT, int dim = 3>
class IVoxNode {
   public:
    IVoxNode() = default;
    IVoxNode(const PointT& center, const float& side_length) {}

    void InsertPoint(const PointT& pt) { points_.emplace_back(pt); }

    template <typename F>
    void ForEachPoint(F&& f) const {
        for (const auto& pt : points_) {
            f(pt);
        }
    }

   private:
    std::vector<PointT> points_;
};

}  // namespace faster_lio

#endif  // FASTER_LIO_IVOX3D_NODE_HPP
