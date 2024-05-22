
#ifndef SMART_SERVER_ENCODER_MPP_H
#define SMART_SERVER_ENCODER_MPP_H

#include "rk_mpi.h"
#include "utils/mpp_env.h"
#include "utils/mpp_mem.h"
#include "utils/mpp_time.h"
#include "utils/mpp_debug.h"
#include "utils/mpp_common.h"

#include "utils/utils.h"
#include "utils/mpi_enc_utils.h"
#include "utils/camera_source.h"
#include "utils/mpp_enc_roi_utils.h"

#include <string>

#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>

typedef struct {
    char *file_input = "/dev/video11";

    RK_U32 fps;
    RK_U32 width;
    RK_U32 height;
    RK_U32 hor_stride;
    RK_U32 ver_stride;
    MppFrameFormat format;
    MppCodingType type;
    MppCodingType type_src;


    RK_S32 frame_num;
    RK_S32 loop_cnt;
    /* -rc */
    RK_S32 rc_mode = 0;
    
    /* -bps */
    RK_S32              bps_target;
    RK_S32              bps_max;
    RK_S32              bps_min;

    /* -fps */
    RK_S32              fps_in_flex;
    RK_S32              fps_in_num;
    RK_S32              fps_in_den;
    RK_S32              fps_out_flex;
    RK_S32              fps_out_num;
    RK_S32              fps_out_den;

    /* -qc */
    RK_S32              qp_init;
    RK_S32              qp_min;
    RK_S32              qp_max;
    RK_S32              qp_min_i;
    RK_S32              qp_max_i;

    /* -g gop mode */
    RK_S32              gop_mode;
    RK_S32              gop_len;
    RK_S32              vi_len;

    /* -v q runtime log disable flag */
    RK_U32              quiet;
    /* -v f runtime fps log flag */
    RK_U32              trace_fps;
    // FpsCalc             fps;
    RK_U32              psnr_en;
    RK_U32              ssim_en;
} MpiEncParam;

typedef struct {
    CamSource *cam_ctx;
    MppEncRoiCtx roi_ctx;

    MppCtx ctx;
    MppApi *mpi;
    RK_S32 chn;

    RK_U32 frm_eos;
    RK_U32 pkt_eos;
    RK_S32 frame_num;
    RK_S32 frame_count;
    RK_U64 stream_size;

    RK_S32 fps;
    RK_U32 width = 1920;
    RK_U32 height = 1080;
    RK_U32 hor_stride;
    RK_U32 ver_stride;
    MppFrameFormat fmt = MPP_FMT_YUV420SP;
    MppCodingType type = MPP_VIDEO_CodingAVC;

    size_t frame_size;
    size_t header_size;
    size_t mdinfo_size;

    MppBufferGroup buf_grp;  //缓存池
    MppBuffer frm_buf;
    MppBuffer pkt_buf;
    MppBuffer md_info;
    MppEncSeiMode sei_mode = MPP_ENC_SEI_MODE_DISABLE;
    MppEncHeaderMode header_mode = MPP_ENC_HEADER_MODE_DEFAULT;

    /* encoder config set */
    MppEncCfg       cfg;
    MppEncOSDPltCfg osd_plt_cfg;
    MppEncOSDPlt    osd_plt;
    MppEncOSDData   osd_data;
    RoiRegionCfg    roi_region;
    MppEncROICfg    roi_cfg;
    
    RK_S32 fps_in_flex;
    RK_S32 fps_in_den;
    RK_S32 fps_in_num;
    RK_S32 fps_out_flex;
    RK_S32 fps_out_den;
    RK_S32 fps_out_num;
    RK_S32 bps;
    RK_S32 bps_max;
    RK_S32 bps_min;
    RK_S32 rc_mode;
    RK_S32 gop_mode;
    RK_S32 gop_len;
    RK_S32 vi_len;

    RK_U32 osd_enable;
    RK_U32 osd_mode;
    RK_U32 split_mode;
    RK_U32 split_arg;
    RK_U32 split_out;

    RK_U32 user_data_enable;
    RK_U32 roi_enable;

    MppEncPrepCfg prep_cfg;
    MppEncRcCfg rc_cfg;
    MppEncCodecCfg codec_cfg;
}MpiEncData;


class EncoderMpp {
public:
    EncoderMpp();
    ~EncoderMpp();

    int Init(MpiEncParam *param);
    void UnInit();
    void Decoder(std::string suffix);
private:
    int ctx_init(MpiEncParam *param);
    int enc_cfg_setup();
    void ctx_deinit();
    int mpp_run(std::string suffix);

private:
    MpiEncData *m_pdata;
};




void encoder_test();

#endif