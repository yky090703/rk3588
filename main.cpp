#include <string.h>
#include <thread>

#include "encoder/encoder_mpp.h"
std::string suffix_ = "50052";

void run(void *pdata)
{
    EncoderMpp *encoder_ = (EncoderMpp *)pdata;
    encoder_->Decoder(suffix_);
}

int main(int argc, char **argv)
{
    std::string file_input = "/dev/video11";
    int width = 640;
    int height = 360;
    int fps = 30;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
 
        if (arg == "--help") {
            std::cout << "Usage: myprogram [options]\n"
                      << "Options:\n"
                      << "  --help           Display this help message\n"
                      << "  --i     <file>   Specify input file\n"
                      << "  --w     <width>  Specify encoder width\n"
                      << "  --h     <height> Specify encoder height\n"
                      << "  --fps   <fps>    Specify encoder fps\n"
                      << "  --ip    <ip>     rtsp server ip\n";
            return 0;
        } else if (arg == "-i") {
            if (i + 1 < argc) {
                file_input = argv[i + 1];
                i++;
            } else {
                std::cerr << "--input option requires an argument.\n";
                break;
            }
        } else if (arg == "-w") {
            if (i + 1 < argc) {
                char *ptr;
                width = strtol(argv[i + 1], &ptr, 10);
                i++;
            } else {
                std::cerr << "--output option requires an argument.\n";
                break;
            }
        } else if (arg == "-h") {
            if (i + 1 < argc) {
                char *ptr;
                height = strtol(argv[i + 1], &ptr, 10);
                i++;
            } else {
                std::cerr << "--output option requires an argument.\n";
                break;
            }
        } else if (arg == "-fps") {
            if (i + 1 < argc) {
                char *ptr;
                fps = strtol(argv[i + 1], &ptr, 10);
                i++;
            } else {
                std::cerr << "--output option requires an argument.\n";
                break;
            }
        }  else {
            std::cerr << "Unknown option: " << arg << "\n";
            break;
        }
    }

    MpiEncParam *param = nullptr;
    param = mpp_calloc(MpiEncParam, 1);
    param->file_input = (char*)file_input.c_str();
    param->width = width;
    param->height = height;
    param->format = MPP_FMT_YUV420SP;
    param->type = MPP_VIDEO_CodingAVC;
    param->fps = fps;
    if (!param->hor_stride)
        param->hor_stride = mpi_enc_width_default_stride(param->width, param->format);
    if (!param->ver_stride)
        param->ver_stride = param->height;

    EncoderMpp *encoder_ = new EncoderMpp();
    encoder_->Init(param);
    printf("img width:%d, height:%d, fps:%d\n", width, height, fps);
    std::thread thread_ = std::thread(run, (void*)encoder_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (thread_.joinable()) thread_.join();

    if (encoder_) {
        delete encoder_;
        encoder_ = NULL;
    }

    mpp_free(param);

    return 0;
}
