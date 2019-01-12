#ifndef MNISTREADER_H
#define MNISTREADER_H

#include "mnistReaderIncludes.h"

std::vector<std::vector<cv::Mat> > read_mnist_cv(const char *image_filename,
                                                const char *label_filename);
                                                
void getKeyIn(int &input, std::atomic<bool> &flag, std::atomic<bool> &changed);

#endif