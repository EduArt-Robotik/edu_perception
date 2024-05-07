#include "edu_perception/h264_decoder.hpp"

namespace eduart {
namespace perception {

H264Decoder::H264Decoder()
{
  avcodec_register_all();
  av_init_packet(&_packet);

  if ((_codec = avcodec_find_decoder(AV_CODEC_ID_H264)) == nullptr) {
    throw std::runtime_error("H264Decoder: can't find decoder for h264.");
  }
  if ((_codec_context = avcodec_alloc_context3(_codec)) == nullptr) {
    throw std::runtime_error("H264Decoder: can't allocate codec context.");
  }
  if (avcodec_open2(_codec_context, _codec, nullptr) < 0) {
    throw std::runtime_error("H264Decoder: could not open codec");
  }
  if ((_frame = av_frame_alloc()) == nullptr) {
    throw std::runtime_error("H264Decoder: can't allocate frame.");
  }
  if ((_frame_converted = av_frame_alloc()) == nullptr) {
    throw std::runtime_error("H264Decoder: can't allocate frame converted.");
  }

  _frame_count = 0;
}

H264Decoder::~H264Decoder()
{
  av_frame_free(&_frame);
  av_frame_free(&_frame_converted);
  avcodec_free_context(&_codec_context);
}

cv::Mat H264Decoder::decode(unsigned char* input_buffer, std::size_t size)
{
  if (size == 0) {
    return cv::Mat();
  }

  _packet.size = size;
  _packet.data = input_buffer;

  int got_frame = 0;
  const int length = avcodec_decode_video2(
    _codec_context, _frame, &got_frame, &_packet
  );

  if (length < 0) {
    throw std::runtime_error("H264Decoder: error occurred while decoding.");
  }

  // allocate memory for output buffer at first processing
  if (_output_buffer == nullptr) {
    int frame_size = avpicture_get_size(
      AV_PIX_FMT_GRAY8, _codec_context->width, _codec_context->height);

    if ((_output_buffer = static_cast<std::uint8_t*>(av_malloc(frame_size))) == nullptr) {
      throw std::runtime_error("H264Decoder: can't allocate output buffer.");
    }

    avpicture_fill(
      reinterpret_cast<AVPicture*>(_frame_converted), _output_buffer,
      AV_PIX_FMT_GRAY8, _codec_context->width, _codec_context->height
    );

    _image_convert_context = sws_getContext(
      _codec_context->width,
      _codec_context->height,
      _codec_context->pix_fmt,
      _codec_context->width,
      _codec_context->height,
      AV_PIX_FMT_GRAY8,
      SWS_BICUBIC,
      nullptr,
      nullptr,
      nullptr
    );
  }

  if (got_frame == 0) {
    return cv::Mat();
  }

  sws_scale(
    _image_convert_context,
    _frame->data,
    _frame->linesize,
    0,
    _codec_context->height,
    _frame_converted->data,
    _frame_converted->linesize
  );

  return cv::Mat(
    _codec_context->height, _codec_context->width, CV_8UC1,
    _frame_converted->data[0], _frame_converted->linesize[0]
  );
}

} // end namespace perception
} // end namespace eduart
