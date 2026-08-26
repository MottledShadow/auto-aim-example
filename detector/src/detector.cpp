#include "detector.hpp"

#include <stdexcept>
#include <utility>

namespace auto_aim::detector
{

namespace
{
// PnP 输入选择：0 = 未分类的配对装甲，1 = 数字分类过滤后的装甲
constexpr int kPnPInputMode = 0;
}

//三件套各自默认参数 + 存下帧源回调
Detector::Detector(FrameSource source)
    : classifier_(NumberClassifierParams{})
    , pnpSolver_(PnpSolverParams{})
    , source_(std::move(source))
{
    //模型/标定任一加载失败即 fail-fast，别让线程空转出不了结果
    if (!error().empty())
    {
        throw std::runtime_error("Detector init failed: " + error());
    }

    //起后台识别线程
    thread_ = std::thread(&Detector::detectLoop, this);
}

Detector::~Detector()
{
    //停线程 → 等它退出
    slot_.stop();
    if (thread_.joinable())
    {
        thread_.join();
    }
}

DetectionResult Detector::detect(const FrameInput& input)
{
    //预处理：灰度→二值→形态学→轮廓候选
    const PreprocessResult pre = lightbarDetector_.preprocess(input.image);

    //筛灯条并定颜色
    const std::vector<LightBar> bars = lightbarDetector_.filterLightBars(input.image, pre);

    //同色灯条配对成装甲
    const std::vector<Armor> armors = lightbarDetector_.matchArmors(bars);

    //数字分类，写回 number/confidence 并按规则去留
    const std::vector<Armor> classified = classifier_.classify(input.image, armors);

    //解位姿
    const std::vector<Armor> solved =
        kPnPInputMode == 0 ? pnpSolver_.solve(armors) : pnpSolver_.solve(classified);

    //打包结果，把取帧时刻的时间戳/四元数原样透传给追踪器
    return DetectionResult{solved, input.timestamp, input.quaternion};
}

void Detector::detectLoop()
{
    while (slot_.running)
    {
        //1. 帧源取一帧，暂时没帧就重来（语义同 Serial 里 read()<=0）
        FrameInput input;
        if (!source_(input))
        {
            continue;
        }

        //2. 整条流水线只在本线程跑（阶段类不可重入）
        DetectionResult result = detect(input);

        //3. 存最新结果，供主线程 latest() 取
        slot_.publish(std::move(result));
    }
}

DetectionResult Detector::latest()
{
    return slot_.latest();
}

} // namespace auto_aim::detector
