#include "encoder_mpp.h"
#include "im2d.h"
#include "rga.h"

static im_rect initial_bbox;
static bool receive_bbox = false;

EncoderMpp::EncoderMpp()
    : m_pdata(NULL)
{
    m_pdata = mpp_calloc(MpiEncData, 1);
}

EncoderMpp::~EncoderMpp()
{
    UnInit();
}

int EncoderMpp::ctx_init(MpiEncParam *param)
{
    MPP_RET ret = MPP_OK;

    // get paramter from cmd
    m_pdata->fps = param->fps;
    m_pdata->width = param->width;
    m_pdata->height = param->height;
    m_pdata->hor_stride = (param->hor_stride) ? (param->hor_stride) : (MPP_ALIGN(param->width, 16));
    m_pdata->ver_stride = (param->ver_stride) ? (param->ver_stride) : (MPP_ALIGN(param->height, 16));
    m_pdata->fmt = param->format;
    m_pdata->type = param->type;

    m_pdata->bps = param->bps_target;
    m_pdata->bps_min = param->bps_min;
    m_pdata->bps_max = param->bps_max;
    m_pdata->rc_mode = param->rc_mode;
    // m_pdata->frame_num    = param->frame_num;
    m_pdata->mdinfo_size = (MPP_VIDEO_CodingHEVC == param->type) ? (MPP_ALIGN(m_pdata->hor_stride, 64) >> 6) *
                                                                       (MPP_ALIGN(m_pdata->ver_stride, 64) >> 6) * 32
                                                                 : (MPP_ALIGN(m_pdata->hor_stride, 64) >> 6) *
                                                                       (MPP_ALIGN(m_pdata->ver_stride, 16) >> 4) * 8;

    if (!strncmp(param->file_input, "/dev/video", 10))
    {
        mpp_log("open camera device");
        // m_pdata->cam_ctx = camera_source_init(param->file_input, 4, m_pdata->width, m_pdata->height, m_pdata->fmt);
        m_pdata->cam_ctx = camera_source_init(param->file_input, 4, m_pdata->width, m_pdata->height, MPP_FMT_YUV420SP);
        mpp_log("new framecap ok");
        if (m_pdata->cam_ctx == NULL)
            mpp_err("open %s fail", param->file_input);
    }

    switch (m_pdata->fmt & MPP_FRAME_FMT_MASK)
    {
    case MPP_FMT_YUV420SP:
    case MPP_FMT_YUV420P:
    {
        m_pdata->frame_size = MPP_ALIGN(m_pdata->hor_stride, 64) * MPP_ALIGN(m_pdata->ver_stride, 64) * 3 / 2;
    }
    break;

    default:
    {
        m_pdata->frame_size = MPP_ALIGN(m_pdata->hor_stride, 64) * MPP_ALIGN(m_pdata->ver_stride, 64) * 4;
        // m_pdata->frame_size = MPP_ALIGN(m_pdata->hor_stride, 64) * MPP_ALIGN(m_pdata->ver_stride, 64) * 3;
    }
    break;
    }

    m_pdata->header_size = 0;

    return ret;
}

int EncoderMpp::enc_cfg_setup()
{
    MPP_RET ret = MPP_OK;
    MppApi *mpi = m_pdata->mpi;
    MppCtx ctx = m_pdata->ctx;
    MppEncCfg cfg = m_pdata->cfg;

    /* setup default parameter */
    m_pdata->fps_in_den = 1;
    m_pdata->fps_in_num = m_pdata->fps;
    m_pdata->fps_out_den = 1;
    m_pdata->fps_out_num = m_pdata->fps;
    m_pdata->bps = m_pdata->width * m_pdata->height * 1.5;
    // m_pdata->bps = m_pdata->bps = m_pdata->width * m_pdata->height / 8 * (m_pdata->fps_out_num / m_pdata->fps_out_den);
    m_pdata->rc_mode = MPP_ENC_RC_MODE_CBR;
    MPP_ENC_RC_QUALITY_MEDIUM;

    mpp_enc_cfg_set_s32(cfg, "prep:width", m_pdata->width);
    mpp_enc_cfg_set_s32(cfg, "prep:height", m_pdata->height);
    mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", m_pdata->hor_stride);
    mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", m_pdata->ver_stride);
    mpp_enc_cfg_set_s32(cfg, "prep:format", m_pdata->fmt);

    mpp_enc_cfg_set_s32(cfg, "rc:mode", m_pdata->rc_mode);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_in_flex", m_pdata->fps_in_flex);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_in_num", m_pdata->fps_in_num);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_in_denorm", m_pdata->fps_in_den);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_out_flex", m_pdata->fps_out_flex);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_out_num", m_pdata->fps_out_num);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_out_denorm", m_pdata->fps_out_den);
    mpp_enc_cfg_set_s32(cfg, "rc:gop", m_pdata->gop_len ? m_pdata->gop_len : m_pdata->fps_out_num * 2);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_target", m_pdata->bps);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_max", m_pdata->bps_max ? m_pdata->bps_max : m_pdata->bps * 17 / 16);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_min", m_pdata->bps_min ? m_pdata->bps_min : m_pdata->bps * 15 / 16);

    /* drop frame or not when bitrate overflow */
    mpp_enc_cfg_set_u32(cfg, "rc:drop_mode", MPP_ENC_RC_DROP_FRM_DISABLED);
    mpp_enc_cfg_set_u32(cfg, "rc:drop_thd", 20); /* 20% of max bps */
    mpp_enc_cfg_set_u32(cfg, "rc:drop_gap", 1);  /* Do not continuous drop frame */

    mpp_enc_cfg_set_s32(cfg, "rc:qp_init", -1);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_max", 51);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_min", 10);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", 51);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", 10);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_ip", 2);

    mpp_enc_cfg_set_s32(cfg, "codec:type", m_pdata->type);
    mpp_enc_cfg_set_s32(cfg, "h264:profile", 100);
    mpp_enc_cfg_set_s32(cfg, "h264:level", 40);
    mpp_enc_cfg_set_s32(cfg, "h264:cabac_en", 1);
    mpp_enc_cfg_set_s32(cfg, "h264:cabac_idc", 0);
    mpp_enc_cfg_set_s32(cfg, "h264:trans8x8", 1);

    ret = mpi->control(ctx, MPP_ENC_SET_CFG, cfg);
    if (ret)
    {
        mpp_err("mpi control enc set cfg failed ret %d\n", ret);
        return ret;
    }

    // 设置帧信息
    m_pdata->sei_mode = MPP_ENC_SEI_MODE_ONE_FRAME;
    ret = mpi->control(ctx, MPP_ENC_SET_SEI_CFG, &m_pdata->sei_mode);
    if (ret)
    {
        mpp_err("mpi control enc set sei cfg failed ret %d\n", ret);
        return ret;
    }

    if (m_pdata->type == MPP_VIDEO_CodingAVC || m_pdata->type == MPP_VIDEO_CodingHEVC)
    {
        m_pdata->header_mode = MPP_ENC_HEADER_MODE_EACH_IDR;
        ret = mpi->control(ctx, MPP_ENC_SET_HEADER_MODE, &m_pdata->header_mode);
        if (ret)
        {
            mpp_err("mpi control enc set header mode failed ret %d\n", ret);
            return ret;
        }
    }

    return ret;
}


void EncoderMpp::ctx_deinit()
{
    if (m_pdata)
    {
        if (m_pdata->cam_ctx)
        {
            camera_source_deinit(m_pdata->cam_ctx);
            m_pdata->cam_ctx = NULL;
        }
    }
}

int EncoderMpp::mpp_run(std::string suffix)
{
    MPP_RET ret = MPP_OK;
    MppApi *mpi = m_pdata->mpi;
    MppCtx ctx = m_pdata->ctx;
    RK_S32 chn = m_pdata->chn;
    RK_U32 cap_num = 0;

    RK_S64 t_s = 0;
    RK_S64 t_e = 0;
    // im_rect show_rect;
    receive_bbox = true;
    struct timeval time;
    gettimeofday(&time, nullptr);
    auto startTime = time.tv_sec * 1000 + time.tv_usec / 1000;
    int frames = 0;
    auto beforeTime = startTime;
    while (!m_pdata->pkt_eos)
    {
        MppFrame frame = NULL;
        MppPacket packet = NULL;
        MppMeta meta = NULL;
        void *buf = mpp_buffer_get_ptr(m_pdata->frm_buf);
        RK_S32 cam_frm_idx = -1;
        MppBuffer cam_buf = NULL;
        RK_U32 eoi = 1;
        t_s = mpp_time();
        cam_frm_idx = camera_source_get_frame(m_pdata->cam_ctx);
        mpp_assert(cam_frm_idx >= 0);
        t_e = mpp_time();
        printf("camera_source_get_frame use time: %lld ms\n", (t_e-t_s)/1000);
        RK_S64 t_1 = mpp_time();
        cam_buf = camera_frame_to_buf(m_pdata->cam_ctx, cam_frm_idx);
        // ###################################################################
        void *rgbBuf = nullptr;
        int frameSize = m_pdata->width * m_pdata->height * 3;
        rgbBuf = malloc(frameSize);
        memset(rgbBuf, 0, frameSize);
        rga_buffer_t rgb_rga = wrapbuffer_virtualaddr((void *)rgbBuf, m_pdata->width, m_pdata->height, RK_FORMAT_RGB_888);
        int dmaFd = mpp_buffer_get_fd(cam_buf);
        rga_buffer_t input_src = wrapbuffer_fd(dmaFd, m_pdata->width, m_pdata->height, RK_FORMAT_YCbCr_420_SP);
        IM_STATUS STATUS;
        STATUS = imcvtcolor(input_src, rgb_rga, RK_FORMAT_YCbCr_420_SP, RK_FORMAT_RGB_888);
        cv::Mat img_show(m_pdata->height, m_pdata->width, CV_8UC3, rgbBuf);
        frames++;
        if (frames % 120 == 0)
        {
            gettimeofday(&time, nullptr);
            auto currentTime = time.tv_sec * 1000 + time.tv_usec / 1000;
            printf("120帧内平均帧率:\t %f fps/s\n", 120.0 / float(currentTime - beforeTime) * 1000.0);
            beforeTime = currentTime;
        }
        cv::imshow("result",img_show);
         if (cv::waitKey(30) >= 0)
        { 
            break;
        }
        // ###################################################################
        
        mpp_assert(cam_buf);

        if (cam_frm_idx >= 0)
            camera_source_put_frame(m_pdata->cam_ctx, cam_frm_idx);

        if (m_pdata->frm_eos && m_pdata->pkt_eos)
            break;
    }

    return ret;
}

int EncoderMpp::Init(MpiEncParam *param)
{
    MPP_RET ret = MPP_OK;
    MppPollType timeout = MPP_POLL_BLOCK;

    ret = (MPP_RET)ctx_init(param);
    if (ret)
    {
        mpp_err_f("test data init failed ret %d\n", ret);
        UnInit();
    }

    // 获取DRM缓存池：internal
    ret = mpp_buffer_group_get_internal(&m_pdata->buf_grp, MPP_BUFFER_TYPE_DRM);
    if (ret)
    {
        mpp_err_f("failed to get mpp buffer group ret %d\n", ret);
        UnInit();
    }

    // 从缓存池分配输入帧存储大小 MppBuffer：frm_buf
    ret = mpp_buffer_get(m_pdata->buf_grp, &m_pdata->frm_buf, m_pdata->frame_size + m_pdata->header_size);
    if (ret)
    {
        mpp_err_f("failed to get buffer for input frame ret %d\n", ret);
        UnInit();
    }

    // 从缓存池分配编码输出存储大小 MppBuffer：pkt_buf
    ret = mpp_buffer_get(m_pdata->buf_grp, &m_pdata->pkt_buf, m_pdata->frame_size);
    if (ret)
    {
        mpp_err_f("failed to get buffer for output packet ret %d\n", ret);
        UnInit();
    }

    // 从缓存池分配运动信息输出存储大小 MppBuffer：md_info
    ret = mpp_buffer_get(m_pdata->buf_grp, &m_pdata->md_info, m_pdata->mdinfo_size);
    if (ret)
    {
        mpp_err_f("failed to get buffer for motion info output packet ret %d\n", ret);
        UnInit();
    }

    // 创建编码器
    ret = mpp_create(&m_pdata->ctx, &m_pdata->mpi);
    if (ret)
    {
        mpp_err("mpp_create failed ret %d\n", ret);
        UnInit();
    }

    // 设置编码超时时间（阻塞）
    ret = m_pdata->mpi->control(m_pdata->ctx, MPP_SET_OUTPUT_TIMEOUT, &timeout);
    if (MPP_OK != ret)
    {
        mpp_err("mpi control set output timeout %d ret %d\n", timeout, ret);
        UnInit();
    }

    // 初始化编码器上下文
    ret = mpp_init(m_pdata->ctx, MPP_CTX_ENC, m_pdata->type);
    if (ret)
    {
        mpp_err("mpp_init failed ret %d\n", ret);
        UnInit();
    }

    // 初始化编码器参数配置
    ret = mpp_enc_cfg_init(&m_pdata->cfg);
    if (ret)
    {
        mpp_err_f("mpp_enc_cfg_init failed ret %d\n", ret);
        UnInit();
    }

    ret = (MPP_RET)enc_cfg_setup();
    if (ret)
    {
        mpp_err_f("test mpp setup failed ret %d\n", ret);
        UnInit();
    }

    return 0;
}

void EncoderMpp::UnInit()
{
    if (m_pdata->ctx)
    {
        mpp_destroy(m_pdata->ctx);
        m_pdata->ctx = NULL;
    }

    if (m_pdata->cfg)
    {
        mpp_enc_cfg_deinit(m_pdata->cfg);
        m_pdata->cfg = NULL;
    }

    if (m_pdata->frm_buf)
    {
        mpp_buffer_put(m_pdata->frm_buf);
        m_pdata->frm_buf = NULL;
    }

    if (m_pdata->pkt_buf)
    {
        mpp_buffer_put(m_pdata->pkt_buf);
        m_pdata->pkt_buf = NULL;
    }

    if (m_pdata->md_info)
    {
        mpp_buffer_put(m_pdata->md_info);
        m_pdata->md_info = NULL;
    }

    if (m_pdata->osd_data.buf)
    {
        mpp_buffer_put(m_pdata->osd_data.buf);
        m_pdata->osd_data.buf = NULL;
    }

    if (m_pdata->buf_grp)
    {
        mpp_buffer_group_put(m_pdata->buf_grp);
        m_pdata->buf_grp = NULL;
    }

    if (m_pdata->roi_ctx)
    {
        mpp_enc_roi_deinit(m_pdata->roi_ctx);
        m_pdata->roi_ctx = NULL;
    }

    ctx_deinit();
}

void EncoderMpp::Decoder(std::string suffix)
{
    MPP_RET ret = MPP_OK;
    MppApi *mpi = m_pdata->mpi;
    MppCtx ctx = m_pdata->ctx;

    ret = (MPP_RET)mpp_run(suffix);
    if (ret)
    {
        mpp_err_f("test mpp run failed ret %d\n", ret);
        UnInit();
    }

    ret = mpi->reset(ctx);
    if (ret)
    {
        mpp_err("mpi->reset failed\n");
        UnInit();
    }
}
