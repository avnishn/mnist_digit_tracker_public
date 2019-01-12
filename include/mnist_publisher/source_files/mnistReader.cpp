#include "mnistReader.h"
#include <stdio.h>

// a helper function when parsing through mnist data
uint32_t swap_endian(uint32_t val) {
  val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
  return (val << 16) | (val >> 16);
}

// function reads each mnist entry into an opencv mat object
// then stores that in a
std::vector<std::vector<cv::Mat>> read_mnist_cv(const char *image_filename,
                                                const char *label_filename) {
  std::vector<std::vector<cv::Mat>> vec(10);

  // Open files
  std::ifstream image_file(image_filename, std::ios::in | std::ios::binary);
  std::ifstream label_file(label_filename, std::ios::in | std::ios::binary);
  if (!image_file.is_open()) {
    std::cout << "broken" << std::endl;
  }
  if (!label_file.is_open()) {
    std::cout << "broken2" << std::endl;
  }
  // Read the magic and the meta data

  uint32_t magic;
  uint32_t num_items;
  uint32_t num_labels;
  uint32_t rows;
  uint32_t cols;

  image_file.read(reinterpret_cast<char *>(&magic), 4);
  magic = swap_endian(magic);
  if (magic != 2051) {
    std::cout << "Incorrect image file magic: " << magic << std::endl;
  }

  label_file.read(reinterpret_cast<char *>(&magic), 4);
  magic = swap_endian(magic);
  if (magic != 2049) {
    std::cout << "Incorrect image file magic: " << magic << std::endl;
  }

  image_file.read(reinterpret_cast<char *>(&num_items), 4);
  num_items = swap_endian(num_items);
  label_file.read(reinterpret_cast<char *>(&num_labels), 4);
  num_labels = swap_endian(num_labels);
  if (num_items != num_labels) {
    std::cout << "image file nums should equal to label num" << std::endl;
  }

  image_file.read(reinterpret_cast<char *>(&rows), 4);
  rows = swap_endian(rows);
  image_file.read(reinterpret_cast<char *>(&cols), 4);
  cols = swap_endian(cols);

  std::cout << "image and label num is: " << num_items << std::endl;
  std::cout << "image rows: " << rows << ", cols: " << cols << std::endl;

  char label;
  char *pixels = new char[rows * cols];
  for (int item_id = 0; item_id < num_items; ++item_id) {
    char *pixels = new char[rows * cols];

    // read image pixel
    image_file.read(pixels, rows * cols);
    // read label
    label_file.read(&label, 1);

    // convert it to cv Mat, and show it

    cv::Mat image_tmp = cv::Mat(rows, cols, CV_8UC1, pixels);
    // resize bigger for showing
    cv::resize(image_tmp, image_tmp, cv::Size(100, 100));

    vec[int(label)].push_back(image_tmp);
  }

  return vec;
}

int getch() {
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt); // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);               // disable buffering
  tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

  int c = getchar();                       // read character (non-blocking)
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
  return c;
}

void getKeyIn(int &input, std::atomic<bool> &flag, std::atomic<bool> &changed) {
  while (flag) {
    std::string user_input(1, (char)getch());
    const char *DIGITS = "0123456789";
    size_t notaDigit = user_input.find_first_not_of(DIGITS);
    if (notaDigit == std::string::npos)
    {
      int user_int = 0;
      std::stringstream ss;
      ss << user_input;
      ss >> user_int;
      input = user_int;
      changed = true;
    }
  }
}
