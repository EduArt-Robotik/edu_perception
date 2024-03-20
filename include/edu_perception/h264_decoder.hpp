/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <opencv2/core/core.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

namespace eduart {
namespace perception {

class H264Decoder
{
public:
  H264Decoder();
  ~H264Decoder();

  inline cv::Mat operator()(unsigned char* input_buffer, std::size_t size) {
    return decode(input_buffer, size);
  }

private:
  cv::Mat decode(unsigned char* input_buffer, std::size_t size);

  AVCodec* _codec;
  AVCodecContext* _codec_context;
  AVFrame* _frame;
  AVFrame* _frame_converted;
  AVPacket _packet;
  int _frame_count;
  std::uint8_t* _output_buffer;
  SwsContext* _image_convert_context;
};

} // end namespace perception
} // end namespace eduart
