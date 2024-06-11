#include <robot_body_filter/utils/tf2_sensor_msgs.h>

#include <robot_body_filter/utils/cloud.h>
#include <robot_body_filter/utils/string_utils.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Geometry>  // needs to be implementation-private as we want -march=native optimizations

#include <unordered_set>

#include <boost/asio.hpp>

namespace robot_body_filter {

const static std::unordered_map<std::string, CloudChannelType> XYZ_CHANNELS({
  {"", CloudChannelType::POINT},
});

const static std::unordered_map<std::string, CloudChannelType> DEFAULT_CHANNELS({
  {"vp_", CloudChannelType::POINT},
  {"normal_", CloudChannelType::DIRECTION}
});

bool fieldNameMatchesChannel(const std::string& fieldName, const std::string& channelName, CloudChannelType channelType)
{
  if (channelType == CloudChannelType::SCALAR) {
    return fieldName == channelName;
  } else if (channelName.empty()) {
    return fieldName == "x" || fieldName == "y" || fieldName == "z";
  } else {
    return fieldName.length() == channelName.length() + 1 && startsWith(fieldName, channelName) && (
        endsWith(fieldName, "x") || endsWith(fieldName, "y") || endsWith(fieldName, "z"));
  }
}

void ApplyTransform(CloudConstIter& x_in, CloudConstIter& y_in, CloudConstIter& z_in, CloudIter& x_out,
                    CloudIter& y_out, CloudIter& z_out, const Eigen::Isometry3f& t, size_t sp, size_t ep) {
  Eigen::Vector3f point;
  for (size_t i = sp; i < ep; ++i) {
    point = t * Eigen::Vector3f(*(x_in + i), *(y_in + i), *(z_in + i));  // apply the whole transform
    *(x_out + i) = point.x();
    *(y_out + i) = point.y();
    *(z_out + i) = point.z();
  }
}

void SegmentTransform(size_t np, size_t threads, CloudConstIter& x_in, CloudConstIter& y_in, CloudConstIter& z_in,
                      CloudIter& x_out, CloudIter& y_out, CloudIter& z_out, const Eigen::Isometry3f& t) {
  //Given a number of points and a number of threads, this function will divide the points into segments
  //Making this threadpool static is causing issues with the transform?
  boost::asio::thread_pool pool(threads);
  for (size_t i = 0; i < threads; ++i) {
    size_t sp = i * np / threads;
    size_t ep = (i + 1) * np / threads;
    boost::asio::post(pool, std::bind(ApplyTransform, std::ref(x_in), std::ref(y_in), std::ref(z_in), std::ref(x_out),
                                      std::ref(y_out), std::ref(z_out), std::ref(t), sp, ep));
  }
  pool.join();
}

void transformChannel(const sensor_msgs::msg::PointCloud2& cloudIn, sensor_msgs::msg::PointCloud2& cloudOut,
                      const Eigen::Isometry3f& t, const std::string& channelPrefix, const CloudChannelType type)
{
  if (num_points(cloudIn) == 0)
    return;
  if (type == CloudChannelType::SCALAR)
    return;
  CloudConstIter x_in(cloudIn, channelPrefix + "x");
  CloudConstIter y_in(cloudIn, channelPrefix + "y");
  CloudConstIter z_in(cloudIn, channelPrefix + "z");

  CloudIter x_out(cloudOut, channelPrefix + "x");
  CloudIter y_out(cloudOut, channelPrefix + "y");
  CloudIter z_out(cloudOut, channelPrefix + "z");

  Eigen::Vector3f point;
  size_t np = num_points(cloudIn);

  // the switch has to be outside the for loop for performance reasons
  switch (type)
  {
    case CloudChannelType::POINT: {
      // for (; x_in != x_in.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out) {
      //   point = t * Eigen::Vector3f(*x_in, *y_in, *z_in);  // apply the whole transform
      //   *x_out = point.x();
      //   *y_out = point.y();
      //   *z_out = point.z();
      // }
      SegmentTransform(np, 16, std::ref(x_in), std::ref(y_in), std::ref(z_in), std::ref(x_out), std::ref(y_out), std::ref(z_out), t);
      break;
    }
    case CloudChannelType::DIRECTION:
      for (; x_out != x_out.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out)
      {
        point = t.linear() * Eigen::Vector3f(*x_in, *y_in, *z_in);  // apply only rotation
        *x_out = point.x();
        *y_out = point.y();
        *z_out = point.z();
      }
      RCLCPP_ERROR(rclcpp::get_logger("robot_body_filter"), "break");
      break;
    case CloudChannelType::SCALAR:
    //TODO: ADD WARNING FOR NOT SUPPORTED
      break;
  }
}

void transformChannel(sensor_msgs::msg::PointCloud2& cloud, const geometry_msgs::msg::Transform& tf,
                      const std::string& channelPrefix, CloudChannelType type)
{
  const auto t = tf2::transformToEigen(tf).cast<float>();
  transformChannel(cloud, cloud, t, channelPrefix, type);
}

sensor_msgs::msg::PointCloud2& transformWithChannels(
    const sensor_msgs::msg::PointCloud2& in, sensor_msgs::msg::PointCloud2& out, const geometry_msgs::msg::TransformStamped& tf)
{
  return transformWithChannels(in, out, tf, DEFAULT_CHANNELS);
}

sensor_msgs::msg::PointCloud2& transformWithChannels(
    const sensor_msgs::msg::PointCloud2& in, sensor_msgs::msg::PointCloud2& out, const geometry_msgs::msg::TransformStamped& tf,
    const std::unordered_map<std::string, CloudChannelType>& channels)
{
  std::unordered_set<std::string> channelsPresent;
  for (const auto& field: in.fields) {
    for (const auto& channelAndType : channels)
    {
      const std::string& channel = channelAndType.first;
      const auto& channelType = channelAndType.second;
      if (channelType != CloudChannelType::SCALAR && fieldNameMatchesChannel(field.name, channel, channelType))
        channelsPresent.insert(channel);
    }
  }


  out = in;
  out.header = tf.header;

  const auto t = tf2::transformToEigen(tf).cast<float>();
  transformChannel(in, out, t, "", CloudChannelType::POINT);
  for (const auto& channel : channelsPresent)
    transformChannel(in, out, t, channel, channels.at(channel));
  return out;
}

sensor_msgs::msg::PointCloud2& transformOnlyChannels(
    const sensor_msgs::msg::PointCloud2& in, sensor_msgs::msg::PointCloud2& out, const geometry_msgs::msg::TransformStamped& tf,
    const std::unordered_map<std::string, CloudChannelType>& channels)
{
  std::unordered_set<std::string> channelsPresent;
  out.point_step = 0;
  for (const auto& field: in.fields) {
    for (const auto& channelAndType : channels)
    {
      const std::string& channel = channelAndType.first;
      const auto& channelType = channelAndType.second;
      if (fieldNameMatchesChannel(field.name, channel, channelType)) {
        channelsPresent.insert(channel);
        out.fields.push_back(field);
        out.fields.back().offset = out.point_step;
        out.point_step += sizeOfPointField(field.datatype);
      }
    }
  }

  out.header = tf.header;
  out.is_dense = in.is_dense;
  out.height = in.height;
  out.width = in.width;
  out.is_bigendian = in.is_bigendian;

  CloudModifier mod(out);
  mod.resize(num_points(in));

  const auto t = tf2::transformToEigen(tf).cast<float>();

  for (const auto& channel : channelsPresent) {
    const auto channelType = channels.at(channel);
    if (channelType != CloudChannelType::SCALAR) {
      transformChannel(in, out, t, channel, channelType);
    } else {
      copyChannelData(in, out, channel);
    }
  }

  return out;
}

sensor_msgs::msg::PointCloud2& transformOnlyXYZ(const sensor_msgs::msg::PointCloud2& in, sensor_msgs::msg::PointCloud2& out,
                                           const geometry_msgs::msg::TransformStamped& tf) {
  return transformOnlyChannels(in, out, tf, XYZ_CHANNELS);
}

sensor_msgs::msg::PointCloud2& transformWithChannels(
    const sensor_msgs::msg::PointCloud2& in, sensor_msgs::msg::PointCloud2& out,
    tf2_ros::Buffer& tfBuffer, const std::string& targetFrame)
{
  return transformWithChannels(in, out, tfBuffer, targetFrame, DEFAULT_CHANNELS);
}

sensor_msgs::msg::PointCloud2& transformWithChannels(
    const sensor_msgs::msg::PointCloud2& in, sensor_msgs::msg::PointCloud2& out,
    tf2_ros::Buffer& tfBuffer, const std::string& targetFrame,
    const std::unordered_map<std::string, CloudChannelType>& channels)
{
  const auto tf = tfBuffer.lookupTransform(targetFrame, in.header.frame_id, in.header.stamp);
  return transformWithChannels(in, out, tf, channels);
}

}