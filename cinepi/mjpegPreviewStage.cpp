#include <thread>
#include <mutex>
#include <vector>
#include <time.h>
#include <unistd.h>

#include <libcamera/stream.h>

#include "core/frame_info.hpp"
#include "core/rpicam_app.hpp"
#include "post_processing_stages/post_processing_stage.hpp"

#include <jpeglib.h>
#include <nadjieb/mjpeg_streamer.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>


using Stream = libcamera::Stream;
using MJPEGStreamer = nadjieb::MJPEGStreamer;

class mjpegStreamStage : public PostProcessingStage
{
public:
    mjpegStreamStage(RPiCamApp *app);
    ~mjpegStreamStage();

    char const *Name() const override;
    void Read(boost::property_tree::ptree const &params) override;
    void Configure() override;
    bool Process(CompletedRequestPtr &completed_request) override;
    void Teardown() override;


private:
    Stream *stream_;
    StreamInfo info_;

    std::shared_ptr<spdlog::logger> console;

    int port_;
    bool running_ = true;

    MJPEGStreamer streamer_;

    void compressToJPEG(libcamera::Span<uint8_t> &inputBuffer, std::vector<uint8_t> &outputBuffer);

    int setup_server_socket(int port);
    void server_thread_func();
};

#define NAME "mjpegPreview"

char const *mjpegStreamStage::Name() const
{
    return NAME;
}

void mjpegStreamStage::compressToJPEG(libcamera::Span<uint8_t> &inputBuffer, std::vector<uint8_t> &outputBuffer)
{
    const bool is_yuv = (stream_->configuration().pixelFormat == libcamera::formats::YUV420);

    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;

    cinfo.dct_method = JDCT_FASTEST;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    cinfo.image_width = info_.width;
    cinfo.image_height = info_.height;
    cinfo.input_components = 3;
    cinfo.in_color_space = is_yuv ? JCS_YCbCr : JCS_RGB;
    cinfo.restart_interval = 0;

    jpeg_set_defaults(&cinfo);
    cinfo.raw_data_in = is_yuv ? TRUE : FALSE;
    jpeg_set_quality(&cinfo, 60, TRUE); // Assuming a quality of 75, adjust as needed.

    uint8_t* encoded_buffer = nullptr;
    unsigned long jpeg_mem_len;
    jpeg_mem_dest(&cinfo, &encoded_buffer, &jpeg_mem_len);

    jpeg_start_compress(&cinfo, TRUE);

    if(!is_yuv){

        int row_stride = info_.stride;  // 3 bytes per pixel for RGB888
        while (cinfo.next_scanline < cinfo.image_height) {
            uint8_t* row_pointer = &inputBuffer.data()[cinfo.next_scanline * row_stride];
            jpeg_write_scanlines(&cinfo, &row_pointer, 1);
        }

    } else {

        int stride2 = info_.stride / 2;
        uint8_t *Y = inputBuffer.data();
        uint8_t *U = Y + info_.stride * info_.height;
        uint8_t *V = U + stride2 * (info_.height / 2);
        uint8_t *Y_max = U - info_.stride;
        uint8_t *U_max = V - stride2;
        uint8_t *V_max = U_max + stride2 * (info_.height / 2);

        JSAMPROW y_rows[16];
        JSAMPROW u_rows[8];
        JSAMPROW v_rows[8];

        for (uint8_t *Y_row = Y, *U_row = U, *V_row = V; cinfo.next_scanline < info_.height;)
        {
            for (int i = 0; i < 16; i++, Y_row += info_.stride)
                y_rows[i] = std::min(Y_row, Y_max);
            for (int i = 0; i < 8; i++, U_row += stride2, V_row += stride2)
                u_rows[i] = std::min(U_row, U_max), v_rows[i] = std::min(V_row, V_max);

            JSAMPARRAY rows[] = { y_rows, u_rows, v_rows };
            jpeg_write_raw_data(&cinfo, rows, 16);
        }
    }

    jpeg_finish_compress(&cinfo);

    // Transfer the encoded buffer to the outputBuffer vector.
    outputBuffer.assign(encoded_buffer, encoded_buffer + jpeg_mem_len);

    // Free the allocated memory.
    free(encoded_buffer);

    jpeg_destroy_compress(&cinfo);
}


void mjpegStreamStage::Read(boost::property_tree::ptree const &params)
{
    port_ = params.get<int>("port", port_);
}

mjpegStreamStage::mjpegStreamStage(RPiCamApp *app) : PostProcessingStage(app)
{
    // Constructor initialization if needed.
    console = spdlog::stdout_color_mt(NAME);
}

mjpegStreamStage::~mjpegStreamStage() 
{
}

void mjpegStreamStage::Teardown(){
    streamer_.stop();
}


void mjpegStreamStage::Configure()
{
    stream_ = app_->GetMainStream();
    info_ = app_->GetStreamInfo(stream_);
    console->info("networkPreviewStage: {}x{} {}", info_.width, info_.height, info_.stride);
    console->info("Setting up NetworkPreview on port: {}", port_);
    streamer_.start(port_, 8);
}

#include <chrono>

bool mjpegStreamStage::Process(CompletedRequestPtr &completed_request)
{
    auto startOverall = std::chrono::high_resolution_clock::now();

    auto startWriteSync = std::chrono::high_resolution_clock::now();
    BufferReadSync r(app_, completed_request->buffers[stream_]);
    auto endWriteSync = std::chrono::high_resolution_clock::now();
    libcamera::Span<uint8_t> buffer = r.Get()[0];

    auto startCompression = std::chrono::high_resolution_clock::now();
    // Compress the buffer using libjpeg-turbo
    std::vector<uint8_t> jpegBuffer;
    compressToJPEG(buffer, jpegBuffer);
    auto endCompression = std::chrono::high_resolution_clock::now();

    console->trace("Sending JPEG buffer size: {}", jpegBuffer.size());

    auto startPublish = std::chrono::high_resolution_clock::now();
    if(streamer_.isRunning()){
        // Publish the JPEG buffer to the streamer
        streamer_.publish("/stream", std::string(jpegBuffer.begin(), jpegBuffer.end()));
    }
    auto endPublish = std::chrono::high_resolution_clock::now();

    auto endOverall = std::chrono::high_resolution_clock::now();

    // Logging the durations
    console->trace("Duration of WriteSync: {} microseconds.", std::chrono::duration_cast<std::chrono::microseconds>(endWriteSync - startWriteSync).count());
    console->trace("Duration of Compression: {} microseconds.", std::chrono::duration_cast<std::chrono::microseconds>(endCompression - startCompression).count());
    console->trace("Duration of Publish: {} microseconds.", std::chrono::duration_cast<std::chrono::microseconds>(endPublish - startPublish).count());
    // console->debug("Overall Duration: {} microseconds.", std::chrono::duration_cast<std::chrono::microseconds>(endOverall - startOverall).count());

    return false;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
    return new mjpegStreamStage(app);
}

static RegisterStage reg(NAME, &Create);