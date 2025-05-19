#include "GaitPhaseDetector.h"
#include <cmath>    // fabsf
#include <limits>   // numeric_limits

/*------------ 构造 & 复位 ------------*/
GaitPhaseDetector::GaitPhaseDetector() { reset(); }

void GaitPhaseDetector::reset()
{
    posSum_ = negSum_ = cumPos_ = cumNeg_ = 0.0f;
    prevPos_ = prevNeg_ = 0.0f;
    hrDone_ = toDone_ = faDone_ = tbvDone_ = false;
}

/*------------ 主算法：单点更新 ------------*/
GaitEvent GaitPhaseDetector::update(float sample)
{
    /* === 1. 积分 === */
    const bool positive = sample > 0.0f;
    const float absVal  = std::fabs(sample);

    if (positive) { posSum_ += sample;   cumPos_ += sample; }
    else          { negSum_ += absVal;  cumNeg_ += absVal; }

    /* === 2. 阈值事件 === */
    if (!positive && !hrDone_ && prevNeg_ > 0 &&
        cumNeg_ >= HR_TH * prevNeg_) { hrDone_ = true; return GaitEvent::HR; }

    if (!positive && !toDone_ && prevNeg_ > 0 &&
        cumNeg_ >= TO_TH * prevNeg_) { toDone_ = true; return GaitEvent::TO; }

    if ( positive && !faDone_ && prevPos_ > 0 &&
        cumPos_ >= FA_TH * prevPos_) { faDone_ = true; return GaitEvent::FA; }

    if ( positive && !tbvDone_ && prevPos_ > 0 &&
        cumPos_ >= TBV_TH * prevPos_) { tbvDone_ = true; return GaitEvent::TBV; }

    /* === 3. IC：周期内第一个局部极小值 === */
    static float prev1 = std::numeric_limits<float>::quiet_NaN();
    static bool  inNeg = false, icDone = false, slopeDown = false;

    if (std::isnan(prev1)) {              // 首样本初始化
        prev1 = sample;
        return GaitEvent::NONE;
    }

    /* 进入负区，重置局部极小检测状态 */
    if (!inNeg && sample < 0.0f) {
        inNeg = true; icDone = false; slopeDown = false;
    }

    /* 在负区寻找“向下→向上”拐点 */
    if (inNeg && !icDone) {
        bool nowDown = sample < prev1;          // 是否继续下降
        if (slopeDown && !nowDown) {            // 出现拐点
            icDone = true;
            prev1 = sample;
            return GaitEvent::IC;               // 立即触发 IC
        }
        slopeDown = nowDown;
    }

    /* 离开负区，准备下一周期 */
    if (inNeg && positive) inNeg = false;

    /* === 4. 周期边界：负→正 零交点 === */
    static bool prevPosFlag = true;
    if (!prevPosFlag && positive) {
        prevPos_ = posSum_;
        prevNeg_ = negSum_;

        posSum_ = negSum_ = cumPos_ = cumNeg_ = 0.0f;
        hrDone_ = toDone_ = faDone_ = tbvDone_ = false;
    }
    prevPosFlag = positive;
    prev1 = sample;                        // 更新 prev1

    return GaitEvent::NONE;
}
