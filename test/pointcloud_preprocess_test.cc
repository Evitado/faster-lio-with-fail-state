#include "pointcloud_preprocess.h"

#include <gtest/gtest.h>
#include <boost/make_shared.hpp>

#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace faster_lio {
namespace {

using sensor_msgs::PointField;

struct InPoint {
    float x = 0, y = 0, z = 0;
    float intensity = 0;
    uint32_t t = 0;  // [ns]
};

struct Field {
    std::string name;
    uint32_t offset;
    uint8_t datatype;
};

/// ouster-like layout: x, y, z, intensity float32 at 0..12, t uint32 at 16
std::vector<Field> DefaultFields() {
    return {{"x", 0, PointField::FLOAT32},
            {"y", 4, PointField::FLOAT32},
            {"z", 8, PointField::FLOAT32},
            {"intensity", 12, PointField::FLOAT32},
            {"t", 16, PointField::UINT32}};
}

/// cloud of width x height points (row-major), each written at the offsets of its fields that exist
sensor_msgs::PointCloud2::Ptr MakeCloud(const std::vector<InPoint> &points, uint32_t width, uint32_t height,
                                        const std::vector<Field> &fields = DefaultFields(), uint32_t point_step = 20,
                                        uint32_t row_padding = 0) {
    auto msg = boost::make_shared<sensor_msgs::PointCloud2>();
    msg->width = width;
    msg->height = height;
    msg->point_step = point_step;
    msg->row_step = width * point_step + row_padding;
    msg->is_bigendian = false;
    for (const auto &f : fields) {
        PointField field;
        field.name = f.name;
        field.offset = f.offset;
        field.datatype = f.datatype;
        field.count = 1;
        msg->fields.push_back(field);
    }
    msg->data.assign(size_t(msg->row_step) * height, 0xAB);  // padding bytes are garbage
    for (uint32_t row = 0; row < height; ++row) {
        for (uint32_t col = 0; col < width; ++col) {
            const InPoint &p = points[row * width + col];
            uint8_t *dst = msg->data.data() + row * msg->row_step + col * point_step;
            for (const auto &f : fields) {
                const void *src = f.name == "x"           ? static_cast<const void *>(&p.x)
                                  : f.name == "y"         ? &p.y
                                  : f.name == "z"         ? &p.z
                                  : f.name == "intensity" ? &p.intensity
                                  : f.name == "t"         ? static_cast<const void *>(&p.t)
                                                          : nullptr;
                if (src && f.offset + 4 <= point_step) std::memcpy(dst + f.offset, src, 4);
            }
        }
    }
    return msg;
}

InPoint In(float x, float y, float z, float intensity = 0, uint32_t t = 0) { return {x, y, z, intensity, t}; }

PointCloudType Preprocess(const sensor_msgs::PointCloud2::ConstPtr &msg, double blind = 0, int filter_num = 1) {
    PointCloudPreprocess preprocess(blind, filter_num);
    CloudPtr out = std::make_shared<PointCloudType>();
    preprocess.Process(msg, out);
    return *out;
}

TEST(PointCloudPreprocess, ConvertsEveryField) {
    const auto msg = MakeCloud({In(1, 2, 3, 40, 5'000'000), In(-4, 5.5f, -6, 0.5f, 12'345'678)}, 2, 1);
    const PointCloudType out = Preprocess(msg);
    ASSERT_EQ(out.size(), 2u);
    EXPECT_EQ(out[0].pos(), Eigen::Vector3f(1, 2, 3));
    EXPECT_EQ(out[0].intensity, 40);
    EXPECT_FLOAT_EQ(out[0].time, 5.0f);  // ns -> ms
    EXPECT_EQ(out[1].pos(), Eigen::Vector3f(-4, 5.5f, -6));
    EXPECT_EQ(out[1].intensity, 0.5f);
    EXPECT_FLOAT_EQ(out[1].time, 12.345678f);
}

TEST(PointCloudPreprocess, EmptyCloud) {
    EXPECT_TRUE(Preprocess(MakeCloud({}, 0, 1)).empty());
    EXPECT_TRUE(Preprocess(MakeCloud({}, 0, 0)).empty());
}

TEST(PointCloudPreprocess, BlindDropsClosePoints) {
    const auto msg = MakeCloud({In(0.5f, 0, 0), In(0, 1.0f, 0), In(0, 0, -1.5f), In(0.6f, 0.6f, 0.6f)}, 4, 1);
    const PointCloudType out = Preprocess(msg, 1.0);
    // range 0.5 and 1.039 vs blind 1: the point at exactly 1 m is kept
    ASSERT_EQ(out.size(), 3u);
    EXPECT_EQ(out[0].pos(), Eigen::Vector3f(0, 1.0f, 0));
    EXPECT_EQ(out[1].pos(), Eigen::Vector3f(0, 0, -1.5f));
    EXPECT_EQ(out[2].pos(), Eigen::Vector3f(0.6f, 0.6f, 0.6f));
}

TEST(PointCloudPreprocess, DropsNonFinitePoints) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const auto msg = MakeCloud({In(nan, nan, nan), In(1, 2, 3), In(0, inf, 0), In(nan, 0, 0), In(4, 5, -inf), In(6, 0, 0)}, 6, 1);
    for (const double blind : {0.0, 0.5}) {
        const PointCloudType out = Preprocess(msg, blind);
        ASSERT_EQ(out.size(), 2u) << "blind " << blind;
        EXPECT_EQ(out[0].pos(), Eigen::Vector3f(1, 2, 3));
        EXPECT_EQ(out[1].pos(), Eigen::Vector3f(6, 0, 0));
    }
}

TEST(PointCloudPreprocess, PointFilterNumKeepsEveryNthRowMajorPoint) {
    // 4 x 3 organized cloud, x = row-major index
    std::vector<InPoint> points;
    for (int i = 0; i < 12; ++i) points.push_back(In(float(i), 1, 0));
    const auto msg = MakeCloud(points, 4, 3);
    for (const int n : {1, 2, 3, 5, 12, 100}) {
        std::vector<float> kept;
        for (const auto &p : Preprocess(msg, 0, n)) kept.push_back(p.x);
        std::sort(kept.begin(), kept.end());
        std::vector<float> expected;
        for (int i = 0; i < 12; i += n) expected.push_back(float(i));
        EXPECT_EQ(kept, expected) << "point_filter_num " << n;
    }
}

TEST(PointCloudPreprocess, PointFilterNumBelowOneKeepsAll) {
    const auto msg = MakeCloud({In(1, 0, 0), In(2, 0, 0), In(3, 0, 0)}, 3, 1);
    EXPECT_EQ(Preprocess(msg, 0, 0).size(), 3u);
    EXPECT_EQ(Preprocess(msg, 0, -4).size(), 3u);
}

TEST(PointCloudPreprocess, OrganizedCloudIsReadColumnByColumn) {
    // 3 columns x 2 rows, every column shares one time stamp (as on an ouster): output in time order
    std::vector<InPoint> points;
    for (uint32_t row = 0; row < 2; ++row) {
        for (uint32_t col = 0; col < 3; ++col) {
            points.push_back(In(float(col), float(row), 1, 0, col * 1'000'000));
        }
    }
    const PointCloudType out = Preprocess(MakeCloud(points, 3, 2));
    ASSERT_EQ(out.size(), 6u);
    for (size_t i = 0; i < out.size(); ++i) {
        EXPECT_EQ(out[i].x, float(i / 2)) << i;  // column
        EXPECT_EQ(out[i].y, float(i % 2)) << i;  // row
        EXPECT_FLOAT_EQ(out[i].time, float(i / 2));
    }
}

TEST(PointCloudPreprocess, ScanEndCountsDroppedPoints) {
    // the latest time is on a blind point and a point the decimation skips: the scan end still has it
    PointCloudPreprocess preprocess(1.0, 2);
    CloudPtr out = std::make_shared<PointCloudType>();
    EXPECT_DOUBLE_EQ(preprocess.Process(MakeCloud({In(5, 0, 0, 0, 1000000), In(0.1f, 0, 0, 0, 98000000)}, 2, 1), out),
                     98.0);
    EXPECT_DOUBLE_EQ(preprocess.Process(MakeCloud({In(5, 0, 0, 0, 1000000), In(6, 0, 0, 0, 97000000)}, 2, 1), out),
                     97.0);
    ASSERT_EQ(out->size(), 1u);
    EXPECT_DOUBLE_EQ(out->front().time, 1.0);
}

TEST(PointCloudPreprocess, ScanEndIgnoresInvalidReturns) {
    // a NaN or inf point has no position, and its time is not trusted either
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    PointCloudPreprocess preprocess(0, 1);
    CloudPtr out = std::make_shared<PointCloudType>();
    EXPECT_DOUBLE_EQ(preprocess.Process(MakeCloud({In(5, 0, 0, 0, 1000000), In(nan, 0, 0, 0, 99000000)}, 2, 1), out),
                     1.0);
    EXPECT_DOUBLE_EQ(preprocess.Process(MakeCloud({In(5, 0, 0, 0, 1000000), In(0, inf, 0, 0, 99000000)}, 2, 1), out),
                     1.0);
    EXPECT_EQ(out->size(), 1u);
}

TEST(PointCloudPreprocess, ScanEndIgnoresTimesPastOneSecond) {
    // one corrupt time must not move the scan end seconds ahead; the point is dropped, the rest kept
    PointCloudPreprocess preprocess(0, 1);
    CloudPtr out = std::make_shared<PointCloudType>();
    EXPECT_DOUBLE_EQ(preprocess.Process(MakeCloud({In(5, 0, 0, 0, 100000000), In(6, 0, 0, 0, 4000000000u),
                                                   In(7, 0, 0, 0, 1000000000)},
                                                  3, 1),
                                        out),
                     1000.0);
    ASSERT_EQ(out->size(), 2u);
    EXPECT_EQ(out->at(0).x, 5);
    EXPECT_EQ(out->at(1).x, 7);
}

TEST(PointCloudPreprocess, ScanEndWithoutTimeIsZero) {
    PointCloudPreprocess preprocess(0, 1);
    CloudPtr out = std::make_shared<PointCloudType>();
    std::vector<Field> fields = DefaultFields();
    fields.pop_back();  // no 't'
    EXPECT_DOUBLE_EQ(preprocess.Process(MakeCloud({In(5, 0, 0, 0, 7000000)}, 1, 1, fields), out), 0.0);
}

TEST(PointCloudPreprocess, MissingIntensityAndTimeAreZero) {
    const std::vector<Field> xyz{{"x", 0, PointField::FLOAT32}, {"y", 4, PointField::FLOAT32}, {"z", 8, PointField::FLOAT32}};
    const PointCloudType out = Preprocess(MakeCloud({In(1, 2, 3, 9, 9'000'000)}, 1, 1, xyz, 12));
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].pos(), Eigen::Vector3f(1, 2, 3));
    EXPECT_EQ(out[0].intensity, 0);
    EXPECT_EQ(out[0].time, 0);
}

TEST(PointCloudPreprocess, TimeOfAnotherTypeIsIgnored) {
    auto fields = DefaultFields();
    fields[4].datatype = PointField::FLOAT32;
    const PointCloudType out = Preprocess(MakeCloud({In(1, 2, 3, 9, 9'000'000)}, 1, 1, fields));
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].time, 0);
    EXPECT_EQ(out[0].intensity, 9);
}

TEST(PointCloudPreprocess, FieldsInAnyOrderAndPadding) {
    // reordered fields, 32 byte points, 8 bytes of padding per row
    const std::vector<Field> fields{{"t", 0, PointField::UINT32},
                                    {"intensity", 4, PointField::FLOAT32},
                                    {"z", 16, PointField::FLOAT32},
                                    {"y", 20, PointField::FLOAT32},
                                    {"x", 24, PointField::FLOAT32}};
    const auto msg = MakeCloud({In(1, 2, 3, 4, 1'000'000), In(5, 6, 7, 8, 2'000'000), In(9, 10, 11, 12, 3'000'000),
                                In(13, 14, 15, 16, 4'000'000)},
                               2, 2, fields, 32, 8);
    const PointCloudType out = Preprocess(msg);
    ASSERT_EQ(out.size(), 4u);
    // column-major: (0,0), (1,0), (0,1), (1,1)
    const float expected_x[] = {1, 9, 5, 13};
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_EQ(out[i].x, expected_x[i]);
        EXPECT_EQ(out[i].y, expected_x[i] + 1);
        EXPECT_EQ(out[i].z, expected_x[i] + 2);
        EXPECT_EQ(out[i].intensity, expected_x[i] + 3);
    }
    EXPECT_FLOAT_EQ(out[1].time, 3.0f);
}

TEST(PointCloudPreprocess, UnsupportedLayoutsGiveAnEmptyCloud) {
    const std::vector<InPoint> points{In(1, 2, 3), In(4, 5, 6)};
    {
        auto msg = MakeCloud(points, 2, 1);
        msg->is_bigendian = true;
        EXPECT_TRUE(Preprocess(msg).empty()) << "big endian";
    }
    {
        auto fields = DefaultFields();
        fields.erase(fields.begin() + 2);
        EXPECT_TRUE(Preprocess(MakeCloud(points, 2, 1, fields)).empty()) << "no z";
    }
    {
        auto fields = DefaultFields();
        fields[0].datatype = PointField::FLOAT64;
        EXPECT_TRUE(Preprocess(MakeCloud(points, 2, 1, fields)).empty()) << "x is float64";
    }
    {
        auto msg = MakeCloud(points, 2, 1);
        msg->fields[1].offset = 18;  // y would read past the point
        EXPECT_TRUE(Preprocess(msg).empty()) << "y outside the point";
    }
    {
        auto msg = MakeCloud(points, 2, 1);
        msg->data.resize(msg->data.size() - 1);
        EXPECT_TRUE(Preprocess(msg).empty()) << "data too short";
    }
    {
        auto msg = MakeCloud(points, 2, 1);
        msg->row_step = msg->point_step * 2 - 1;
        EXPECT_TRUE(Preprocess(msg).empty()) << "row_step shorter than a row";
    }
}

TEST(PointCloudPreprocess, ReusedOutputHasNoLeftovers) {
    PointCloudPreprocess preprocess(0, 1);
    CloudPtr out = std::make_shared<PointCloudType>();
    preprocess.Process(MakeCloud({In(1, 0, 0), In(2, 0, 0), In(3, 0, 0)}, 3, 1), out);
    ASSERT_EQ(out->size(), 3u);
    preprocess.Process(MakeCloud({In(7, 0, 0)}, 1, 1), out);
    ASSERT_EQ(out->size(), 1u);
    EXPECT_EQ(out->front().x, 7);

    // an unsupported cloud after a good one: empty, not the previous result
    auto bad = MakeCloud({In(1, 0, 0)}, 1, 1);
    bad->is_bigendian = true;
    preprocess.Process(bad, out);
    EXPECT_TRUE(out->empty());
}

}  // namespace
}  // namespace faster_lio

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::Time::init();  // the throttled log macros read the time
    return RUN_ALL_TESTS();
}
