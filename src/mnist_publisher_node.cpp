#include "mnist_publisher/source_files/mnistReader.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// initializes and runs node that is used for publishing a random mnist digit based on
// the corresponding number that is entered into the keyboad

int main(int argc, char *argv[]) {

  // read mnist images into vector of opencv mats
  std::string currentPath = ros::package::getPath("mnist_digit_tracker");
  std::string label_filename = currentPath + \
                              "/include/mnist_publisher/mnist_images/t10k-labels-idx1-ubyte";
  std::string image_filename = currentPath + \
                              "/include/mnist_publisher/mnist_images/t10k-images-idx3-ubyte";
  std::vector<std::vector<cv::Mat>> imagesVec = read_mnist_cv(image_filename.c_str(), label_filename.c_str());

  // initialize image publisher node to broadcast an Image transport message
  // on the mnist_image topic
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("mnist_image", 1);
  ros::Rate loop_rate(5);

  int in = 0;
  int inprev = 0;
  int randElement = 0;

  std::atomic<bool> ready(true);
  std::atomic<bool> changed(false);
  std::thread th1(getKeyIn, std::ref(in), std::ref(ready), std::ref(changed));

  while (nh.ok()) {
    if (changed) {
      // randomize the image that is sent
      randElement = std::rand() % (imagesVec[in].size());
      inprev = in;
      changed = false;
    }
    cv::Mat image = imagesVec[in][randElement];
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  // cleanup after exit
  if (!nh.ok()) {
    ready = false;
    th1.join();
  }

  return 0;
}